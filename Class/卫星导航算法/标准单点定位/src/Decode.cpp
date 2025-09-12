#include"Decode.h"
#include"Error.h"
#include"const.h"
#include"CoordinateConvert.h"
#include<iostream>
#include<fstream>
#include<string>

using namespace std;
extern int mode;


unsigned int UI4(unsigned char* p)    //无符号整型unsigned int
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
int I4(unsigned char* p)    //有符号整型int
{
	int u;
	memcpy(&u, p, 4);
	return u;
}
double D8(unsigned char* p)    //双精度浮点型double
{
	double u;
	memcpy(&u, p, 8);
	return u;
}
short I2(unsigned char* p)    //短整型short
{
	short u;
	memcpy(&u, p, 2);
	return u;
}
float F4(unsigned char* p)   //单精度浮点型float
{
	float u;
	memcpy(&u, p, 4);
	return u;
}
unsigned short UI2(unsigned char* p)     //无符号短整型unsigned short
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}

/**
* @brief 计算CRC32校验码
* @param buff 输入数据缓冲区
* @param len 输入数据长度
* @return 返回计算得到的CRC32校验码
*/
unsigned int Crc32(unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;    //初始化CRC值为0
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];    //将当前字节与CRC值进行异或操作
		for (j = 0; j < 8; j++)    //对每个字节的8位进行处理
		{
			if (crc & 1)    //如果CRC的最低位为1
				crc = (crc >> 1) ^ POLYCRC32;    // 右移1位并与多项式进行异或
			else
				crc >>= 1;    // 如果最低位为0，则仅右移1位
		}
	}
	return crc;    //返回计算得到的CRC值
}

/**
* @brief 解码观测值数据函数
* @param data 输入的字节数据缓冲区
* @param len 输入数据的长度
* @param obs 输出的观测数据结构体
*/
void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs)
{
	int i, j, n = 0, k = 0, ObsNum, Freq, Prn;
	GNSSsys sys;
	double wl;
	//相位锁定标志PhaseLockFlag，码锁定标志CodeLockedFlag，奇偶校验标志ParityFlag，卫星系统SatSystem，信号类型SigType
	int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
	unsigned int ChanStatus;
	unsigned char* p = data + 28;
	//从消息头中解码得到观测时刻，该时刻为接收机钟表面时，用GPSTIME结构体表示。
	obs->Time.Week = UI2(data + 14);
	obs->Time.SecOfWeek = UI4(data + 16) * 1E-3;
	//解码得到观测值数量，为所有卫星所有信号观测值的总数。
	ObsNum = UI4(p);  // 观测值数量
	memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
	//对所有信号观测值进行循环解码
	for (i = 0, p += 4; i < ObsNum; i++, p += 44) {
		//解码得到跟踪状态标记，从中取出Phase lock flag / Code locked
		//flag / Parity known flag / Satellite system / signal type等数据
		ChanStatus = UI4(p + 40);
		ParityFlag = (ChanStatus >> 11) & 0x01;     //奇偶校验
		PhaseLockFlag = (ChanStatus >> 10) & 0x01;    //相位锁定
		CodeLockedFlag = (ChanStatus >> 12) & 0x01;    //码锁定
		SatSystem = (ChanStatus >> 16) & 0x07;    //卫星系统
		SigType = (ChanStatus >> 21) & 0x1F;    //信号系统
	//如果卫星系统不是GPS或BDS， continue
	//如果GPS卫星的信号类型不是L1 C / A或者L2P（Y）， BDS卫星不是B1I或B3I， continue，并记录信号频率类型，第一频率s = 0，第二频率s = 1；
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
		//解码得到卫星号Prn以及卫星系统号，在当前观测值结构体中进行搜索，
		//如果找到相同的卫星，将解码的观测值填充到该卫星对应的数组中；如果在当前已解码的卫星数据中没有发现，则填充到现有数据的末尾。
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
		obs->SatObs[n].L[Freq] = -wl * (PhaseLockFlag == 1 ? D8(p + 16) : 0.0);    //相位观测值单位为周
		obs->SatObs[n].D[Freq] = -wl * F4(p + 28);    //多普勒单位为Hz
		obs->SatObs[n].cn0[Freq] = F4(p + 32);
		obs->SatObs[n].LockTime[Freq] = F4(p + 36);
		obs->SatObs[n].half[Freq] = ParityFlag;
	}
	obs->SatNum = k + 1;
}


/**
* @brief 解码北斗卫星历元数据
* @param data 输入的字节数据缓冲区
* @param len 输入数据的长度
* @param beph 输出的北斗卫星历元数据数组
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
	eph->SVHealth = UI4(p + 16);    //卫星健康状态
	eph->TGD1 = D8(p + 20);
	eph->TGD2 = D8(p + 28);
	eph->IODC = UI4(p + 36);
	eph->TOC.SecOfWeek = UI4(p + 40);
	eph->ClkBias = D8(p + 44);
	eph->ClkDrift = D8(p + 52);
	eph->ClkDriftRate = D8(p + 60);
	eph->IODE = UI4(p + 68);
	eph->TOE.SecOfWeek = UI4(p + 72);
	eph->SqrtA = D8(p + 76);    // 平方根半长轴
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
* @brief 解码GPS卫星历元数据
* @param data 输入的字节数据缓冲区
* @param len 输入数据的长度
* @param geph 输出的GPS卫星历元数据数组
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
	eph->SVHealth = UI4(p + 12);    //卫星健康状态
	eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
	eph->TOE.SecOfWeek = D8(p + 32);
	eph->SqrtA = sqrt(D8(p + 40));    //在信息流中解码得到的是半长轴，需要加根号
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
* @brief 解码定位结果数据
* @param data 输入的字节数据缓冲区
* @param len 输入数据的长度
* @param pos 输出的定位结果结构体
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
* @brief 解码NovAtel OEM7格式的数据
* @param buff 输入的字节数据缓冲区
* @param len 输入数据的长度
* @param obs 输出的观测数据结构体
* @param geph 输出的GPS卫星历元数据数组
* @param beph 输出的北斗卫星历元数据数组
* @param pos 输出的定位结果结构体
* @return 返回解码状态，0表示成功，非0表示失败
*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos)
{
	int i, MsgLen, MsgId, Status;  // 记录待处理数据的起始位置
	i = 0;
	while (1)
	{
		//1. 设置循环变量i = 0，开始查找AA 44 12同步字符
		for (; i < len - 2; i++) {
			if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12) break;
		}
		//2. 找到同步字符后，获取消息头长度的字符28字节， 若字节数量不足即i + 28 > len，跳出循环（break）至第6步
		if (i + 28 > len) break;
		//3. 从消息头中解码消息长度MsgLen和消息类型MsgID，获得整条消息buff[i, i + 28 + MsgLen + 4]，若字节数量不足即i + 28 + MsgLen + 4 > len，跳出循环（break） 至第6步
		MsgId = UI2(buff + i + 4);
		MsgLen = UI2(buff + i + 8);
		if (i + 28 + MsgLen + 4 > len) break;
		//4. CRC检验，若不通过，跳过同步字符3个字节，即i = i + 3，返回到第1步
		if (Crc32(buff + i, 28 + MsgLen) != UI4(buff + i + 28 + MsgLen))
		{
			i += 3;  // 跳过当前AA 44 12
			continue;
		}
		//5. 根据消息ID，调用对应的解码函数，若解码得到观测值，跳出循环至第6步；否则，跳过整条消息，即i = i + 28 + MsgLen + 4，返回第1步
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
	//循环结束后，将不足一条消息的剩余字节，拷贝至buff缓冲区的开始处，将len设置为剩余字节数量，并返回给主函数。
	//memcpy(a,b,c)  a:目的地址，b:源地址，c:拷贝的字节数
	memcpy(buff + 0, buff + i, len - i);
	len = len - i;
	return 0;
}