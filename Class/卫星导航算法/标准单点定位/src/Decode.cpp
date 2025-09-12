#include"Decode.h"
#include"Error.h"
#include"const.h"
#include"CoordinateConvert.h"
#include<iostream>
#include<fstream>
#include<string>

using namespace std;
extern int mode;


unsigned int UI4(unsigned char* p)    //�޷�������unsigned int
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
int I4(unsigned char* p)    //�з�������int
{
	int u;
	memcpy(&u, p, 4);
	return u;
}
double D8(unsigned char* p)    //˫���ȸ�����double
{
	double u;
	memcpy(&u, p, 8);
	return u;
}
short I2(unsigned char* p)    //������short
{
	short u;
	memcpy(&u, p, 2);
	return u;
}
float F4(unsigned char* p)   //�����ȸ�����float
{
	float u;
	memcpy(&u, p, 4);
	return u;
}
unsigned short UI2(unsigned char* p)     //�޷��Ŷ�����unsigned short
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}

/**
* @brief ����CRC32У����
* @param buff �������ݻ�����
* @param len �������ݳ���
* @return ���ؼ���õ���CRC32У����
*/
unsigned int Crc32(unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;    //��ʼ��CRCֵΪ0
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];    //����ǰ�ֽ���CRCֵ����������
		for (j = 0; j < 8; j++)    //��ÿ���ֽڵ�8λ���д���
		{
			if (crc & 1)    //���CRC�����λΪ1
				crc = (crc >> 1) ^ POLYCRC32;    // ����1λ�������ʽ�������
			else
				crc >>= 1;    // ������λΪ0���������1λ
		}
	}
	return crc;    //���ؼ���õ���CRCֵ
}

/**
* @brief ����۲�ֵ���ݺ���
* @param data ������ֽ����ݻ�����
* @param len �������ݵĳ���
* @param obs ����Ĺ۲����ݽṹ��
*/
void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs)
{
	int i, j, n = 0, k = 0, ObsNum, Freq, Prn;
	GNSSsys sys;
	double wl;
	//��λ������־PhaseLockFlag����������־CodeLockedFlag����żУ���־ParityFlag������ϵͳSatSystem���ź�����SigType
	int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
	unsigned int ChanStatus;
	unsigned char* p = data + 28;
	//����Ϣͷ�н���õ��۲�ʱ�̣���ʱ��Ϊ���ջ��ӱ���ʱ����GPSTIME�ṹ���ʾ��
	obs->Time.Week = UI2(data + 14);
	obs->Time.SecOfWeek = UI4(data + 16) * 1E-3;
	//����õ��۲�ֵ������Ϊ�������������źŹ۲�ֵ��������
	ObsNum = UI4(p);  // �۲�ֵ����
	memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
	//�������źŹ۲�ֵ����ѭ������
	for (i = 0, p += 4; i < ObsNum; i++, p += 44) {
		//����õ�����״̬��ǣ�����ȡ��Phase lock flag / Code locked
		//flag / Parity known flag / Satellite system / signal type������
		ChanStatus = UI4(p + 40);
		ParityFlag = (ChanStatus >> 11) & 0x01;     //��żУ��
		PhaseLockFlag = (ChanStatus >> 10) & 0x01;    //��λ����
		CodeLockedFlag = (ChanStatus >> 12) & 0x01;    //������
		SatSystem = (ChanStatus >> 16) & 0x07;    //����ϵͳ
		SigType = (ChanStatus >> 21) & 0x1F;    //�ź�ϵͳ
	//�������ϵͳ����GPS��BDS�� continue
	//���GPS���ǵ��ź����Ͳ���L1 C / A����L2P��Y���� BDS���ǲ���B1I��B3I�� continue������¼�ź�Ƶ�����ͣ���һƵ��s = 0���ڶ�Ƶ��s = 1��
		if (SatSystem == 0) {
			sys = GPS;
			if (SigType == 0) {
				Freq = 0; wl = WL1_GPS;    //L1 C/A
			}
			else if (SigType == 9) { Freq = 1; wl = WL2_GPS; }    //L2P(Y)
			else continue;
		}
		else if (SatSystem == 4) {
			sys = BDS;
			if (SigType == 0 || SigType == 4) {
				Freq = 0; wl = WL1_BDS;    //B1I
			}
			else if (SigType == 2 || SigType == 6) { Freq = 1; wl = WL3_BDS; }    //B3I
			else continue;
		}
		else continue;
		//����õ����Ǻ�Prn�Լ�����ϵͳ�ţ��ڵ�ǰ�۲�ֵ�ṹ���н���������
		//����ҵ���ͬ�����ǣ�������Ĺ۲�ֵ��䵽�����Ƕ�Ӧ�������У�����ڵ�ǰ�ѽ��������������û�з��֣�����䵽�������ݵ�ĩβ��
		Prn = UI2(p);
		for (j = 0; j < MAXCHANNUM; j++) {
			if (obs->SatObs[j].System == sys && obs->SatObs[j].Prn == Prn) {
				n = j; break;
			}
			if (obs->SatObs[j].Prn == 0) {
				k = n = j; break;
			}
		}
		obs->SatObs[n].Prn = Prn;
		obs->SatObs[n].System = sys;
		obs->SatObs[n].P[Freq] = CodeLockedFlag == 1 ? D8(p + 4) : 0.0;
		obs->SatObs[n].L[Freq] = -wl * (PhaseLockFlag == 1 ? D8(p + 16) : 0.0);    //��λ�۲�ֵ��λΪ��
		obs->SatObs[n].D[Freq] = -wl * F4(p + 28);    //�����յ�λΪHz
		obs->SatObs[n].cn0[Freq] = F4(p + 32);
		obs->SatObs[n].LockTime[Freq] = F4(p + 36);
		obs->SatObs[n].half[Freq] = ParityFlag;
	}
	obs->SatNum = k + 1;
}


/**
* @brief ���뱱��������Ԫ����
* @param data ������ֽ����ݻ�����
* @param len �������ݵĳ���
* @param beph ����ı���������Ԫ��������
*/
void DecodeBdsEph(unsigned char* data, int len, GPSEPHREC beph[])
{
	int prn;
	unsigned char* p = data + 28;
	GPSEPHREC* eph;
	prn = UI4(p);
	if (prn < 1 && prn >= MAXBDSNUM) return;
	eph = beph + prn - 1;
	eph->PRN = prn;
	eph->System = BDS;
	eph->TOC.Week = eph->TOE.Week = UI4(p + 4);
	eph->SVHealth = UI4(p + 16);    //���ǽ���״̬
	eph->TGD1 = D8(p + 20);
	eph->TGD2 = D8(p + 28);
	eph->IODC = UI4(p + 36);
	eph->TOC.SecOfWeek = UI4(p + 40);
	eph->ClkBias = D8(p + 44);
	eph->ClkDrift = D8(p + 52);
	eph->ClkDriftRate = D8(p + 60);
	eph->IODE = UI4(p + 68);
	eph->TOE.SecOfWeek = UI4(p + 72);
	eph->SqrtA = D8(p + 76);    // ƽ�����볤��
	eph->e = D8(p + 84);
	eph->omega = D8(p + 92);
	eph->DeltaN = D8(p + 100);
	eph->M0 = D8(p + 108);
	eph->OMEGA = D8(p + 116);
	eph->OMEGADot = D8(p + 124);
	eph->i0 = D8(p + 132);
	eph->iDot = D8(p + 140);
	eph->Cuc = D8(p + 148);
	eph->Cus = D8(p + 156);
	eph->Crc = D8(p + 164);
	eph->Crs = D8(p + 172);
	eph->Cic = D8(p + 180);
	eph->Cis = D8(p + 188);
}

/**
* @brief ����GPS������Ԫ����
* @param data ������ֽ����ݻ�����
* @param len �������ݵĳ���
* @param geph �����GPS������Ԫ��������
*/
void DecodeGpsEph(unsigned char* data, int len, GPSEPHREC geph[])
{
	int prn;
	unsigned char* p = data + 28;
	GPSEPHREC* eph;
	prn = UI4(p);
	if (prn < 1 && prn >= MAXGPSNUM) return;
	eph = geph + prn - 1;
	eph->PRN = prn;
	eph->System = GPS;
	eph->SVHealth = UI4(p + 12);    //���ǽ���״̬
	eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
	eph->TOE.SecOfWeek = D8(p + 32);
	eph->SqrtA = sqrt(D8(p + 40));    //����Ϣ���н���õ����ǰ볤�ᣬ��Ҫ�Ӹ���
	eph->DeltaN = D8(p + 48);
	eph->M0 = D8(p + 56);
	eph->e = D8(p + 64);
	eph->omega = D8(p + 72);
	eph->Cuc = D8(p + 80);
	eph->Cus = D8(p + 88);
	eph->Crc = D8(p + 96);
	eph->Crs = D8(p + 104);
	eph->Cic = D8(p + 112);
	eph->Cis = D8(p + 120);
	eph->i0 = D8(p + 128);
	eph->iDot = D8(p + 136);
	eph->OMEGA = D8(p + 144);
	eph->OMEGADot = D8(p + 152);
	eph->IODC = UI4(p + 160);
	eph->TOC.SecOfWeek = D8(p + 164);
	eph->TGD1 = D8(p + 172);
	eph->ClkBias = D8(p + 180);
	eph->ClkDrift = D8(p + 188);
	eph->ClkDriftRate = D8(p + 196);
}

/**
* @brief ���붨λ�������
* @param data ������ֽ����ݻ�����
* @param len �������ݵĳ���
* @param pos ����Ķ�λ����ṹ��
*/
void DecodePsrPos(unsigned char* data, int len, POSRES* pos)
{
	GEOCOOR geo;
	XYZ xyz;
	unsigned char* p = data + 28;
	geo.latitude = D8(p + 8)*PI/180;
	geo.longitude = D8(p + 16)*PI/180;
	double a = D8(p + 24);
	double b = F4(p + 32);
	geo.height = D8(p + 24) + F4(p + 32);
	BLHToXYZ(geo, xyz, R_WGS84, E_WGS84);
	pos->Pos[0] = xyz.x;
	pos->Pos[1] = xyz.y;
	pos->Pos[2] = xyz.z;
}

/**
* @brief ����NovAtel OEM7��ʽ������
* @param buff ������ֽ����ݻ�����
* @param len �������ݵĳ���
* @param obs ����Ĺ۲����ݽṹ��
* @param geph �����GPS������Ԫ��������
* @param beph ����ı���������Ԫ��������
* @param pos ����Ķ�λ����ṹ��
* @return ���ؽ���״̬��0��ʾ�ɹ�����0��ʾʧ��
*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos)
{
	int i, MsgLen, MsgId, Status;  // ��¼���������ݵ���ʼλ��
	i = 0;
	while (1)
	{
		//1. ����ѭ������i = 0����ʼ����AA 44 12ͬ���ַ�
		for (; i < len - 2; i++) {
			if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12) break;
		}
		//2. �ҵ�ͬ���ַ��󣬻�ȡ��Ϣͷ���ȵ��ַ�28�ֽڣ� ���ֽ��������㼴i + 28 > len������ѭ����break������6��
		if (i + 28 > len) break;
		//3. ����Ϣͷ�н�����Ϣ����MsgLen����Ϣ����MsgID�����������Ϣbuff[i, i + 28 + MsgLen + 4]�����ֽ��������㼴i + 28 + MsgLen + 4 > len������ѭ����break�� ����6��
		MsgId = UI2(buff + i + 4);
		MsgLen = UI2(buff + i + 8);
		if (i + 28 + MsgLen + 4 > len) break;
		//4. CRC���飬����ͨ��������ͬ���ַ�3���ֽڣ���i = i + 3�����ص���1��
		if (Crc32(buff + i, 28 + MsgLen) != UI4(buff + i + 28 + MsgLen))
		{
			i += 3;  // ������ǰAA 44 12
			continue;
		}
		//5. ������ϢID�����ö�Ӧ�Ľ��뺯����������õ��۲�ֵ������ѭ������6������������������Ϣ����i = i + 28 + MsgLen + 4�����ص�1��
		Status = 0;
		switch (MsgId)
		{
		case 43:
			DecodeRange(buff + i, MsgLen, obs);
			Status = 1;
			break;
		case 7:
			DecodeGpsEph(buff + i, MsgLen, geph);
			break;
		case 1696:
			DecodeBdsEph(buff + i, MsgLen, beph);
			break;
		case 42:
			DecodePsrPos(buff + i, MsgLen, pos);
			break;
		default:
			printf("Unknown message ID: 0x%04X\n", MsgId);
			break;
		}
		i = i + 28 + MsgLen + 4;
		if (Status == 1 && mode == 1) break;
	}
	//ѭ�������󣬽�����һ����Ϣ��ʣ���ֽڣ�������buff�������Ŀ�ʼ������len����Ϊʣ���ֽ������������ظ���������
	//memcpy(a,b,c)  a:Ŀ�ĵ�ַ��b:Դ��ַ��c:�������ֽ���
	memcpy(buff + 0, buff + i, len - i);
	len = len - i;
	return 0;
}