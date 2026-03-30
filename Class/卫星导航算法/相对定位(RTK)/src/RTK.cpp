#include<iostream>
#include<fstream>
#include<iomanip>
#include"RTK.h"
#include"Decode.h"
#include"Error.h"
#include"Matrix.h"
#include"sockets.h"
#include"iomanip"

using namespace std;


/**
* @brief 观测数据时间同步函数
* @param FObs_base 基站观测文件指针
* @param FObs_rover 流动站观测文件指针
* @param rawdata 原始数据结构体指针
*/
int TimeSyn(FILE* FObs_base, FILE* FObs_rover, RAWDATA* rawdata)
{
	static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
	static int lenD_base = 0, lenD_rover = 0;
	double dt = 0;
	int lenR_base, lenR_rover;
	while (!feof(FObs_rover))
	{
		if ((lenR_rover = fread(buff_rover + lenD_rover, sizeof(unsigned char), MAXRAWLEN - lenD_rover, FObs_rover)) < MAXRAWLEN - lenD_rover) return -1;
		lenD_rover += lenR_rover;
		if (DecodeNovOem7Dat(buff_rover, lenD_rover, &rawdata->RoverObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestRoverPos) == 1)break;
	}
	dt = (rawdata->RoverObs.Time.Week - rawdata->BaseObs.Time.Week) * 604800.0 + rawdata->RoverObs.Time.SecOfWeek - rawdata->BaseObs.Time.SecOfWeek;

	if (abs(dt) < 0.5)return 1;
	while(!feof(FObs_base))
	{
		if ((lenR_base = fread(buff_base + lenD_base, sizeof(unsigned char), MAXRAWLEN - lenD_base, FObs_base)) < MAXRAWLEN - lenD_base) return -1;
		lenD_base += lenR_base;
		if (DecodeNovOem7Dat(buff_base, lenD_base, &rawdata->BaseObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestPos) == 1)
		{
			dt = (rawdata->RoverObs.Time.Week - rawdata->BaseObs.Time.Week) * 604800.0 + rawdata->RoverObs.Time.SecOfWeek - rawdata->BaseObs.Time.SecOfWeek;
			if (abs(dt) < 0.5)return 1;
			else if (dt < -0.5)return 0;
			else if (dt > 0.5)continue;
		}
	}
}

int RealTimeSyn(SOCKET* NetGps1, SOCKET* NetGps2, RAWDATA* rawdata)
{
	static int lenD_base = 0, lenD_rover = 0;
	int lenR_base, lenR_rover;
	static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
	//*******
	/*while (true)
	{
		if ((lenR_rover = recv(*NetGps1, (char*)buff_rover + lenD_rover, MAXRAWLEN - lenD_rover, 0)) == SOCKET_ERROR) {
			int error = WSAGetLastError();
			if (error == WSAEWOULDBLOCK) {
				Sleep(10);
				continue;
			}	
			printf("Receive error: %d\n", error);
			break;
		}
		else if (lenR_rover == 0) {
			printf("Server closed connection.\n");
			break;
		}
		
		lenD_rover += lenR_rover;
		DecodeNovOem7Dat(buff_rover, lenD_rover, &rawdata->RoverObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestRoverPos);
		double dt = (rawdata->RoverObs.Time.Week - rawdata->BaseObs.Time.Week) * 604800.0 + rawdata->RoverObs.Time.SecOfWeek - rawdata->BaseObs.Time.SecOfWeek;
		if (abs(dt) < 0.5)return 1;
		else
		{
			while (abs(dt) > 0.5)
			{
				cout << "here" << endl;
				if ((lenR_base = recv(*NetGps2, (char*)buff_base + lenD_base, MAXRAWLEN - lenD_base, 0)) == SOCKET_ERROR) {
					int error = WSAGetLastError();
					if (error == WSAEWOULDBLOCK) {
						Sleep(10);
						continue;
					}
					printf("Receive error: %d\n", error);
					break;
				}
				else if (lenR_base == 0) {
					printf("Server closed connection.\n");
					break;
				}
				lenD_base += lenR_base;
				DecodeNovOem7Dat(buff_base, lenD_base, &rawdata->BaseObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestPos);
				dt = (rawdata->RoverObs.Time.Week - rawdata->BaseObs.Time.Week) * 604800.0 + rawdata->RoverObs.Time.SecOfWeek - rawdata->BaseObs.Time.SecOfWeek;
				if (dt < 0)return 0;
			}
			return 1;
		}
	}
	return -1;*/
	//******
	if ((lenR_base = recv(*NetGps2, (char*)buff_base, MAXRAWLEN, 0)) > 0)
	{
		memcpy(buff_base + lenD_base, buff_base, lenR_base);
		lenD_base += lenR_base;
		DecodeNovOem7Dat(buff_base, lenD_base, &rawdata->BaseObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestPos);
	}
	if ((lenR_rover = recv(*NetGps1, (char*)buff_rover, MAXRAWLEN, 0)) > 0)
	{
		memcpy(buff_rover + lenD_rover, buff_rover, lenR_rover);
		lenD_rover += lenR_rover;
		DecodeNovOem7Dat(buff_rover, lenD_rover, &rawdata->RoverObs, rawdata->GPSEph, rawdata->BDSEph, &rawdata->bestRoverPos);
	}
	double dt = (rawdata->RoverObs.Time.Week - rawdata->BaseObs.Time.Week) * 604800.0 + rawdata->RoverObs.Time.SecOfWeek - rawdata->BaseObs.Time.SecOfWeek;
	cout << dt<<"  ";
	if (abs(dt) <= 20)return 1;
	else if (dt < -20)return 0;
	else return -1;
}


/**
* @brief 构建站间单差观测值函数
* @param EpkB 基站历元观测数据指针
* @param EpkR 流动站历元观测数据指针
* @param SDObs 站间单差观测数据指针
*/
void FormSDEpochObs(const EPOCHOBSDATA* EpkB, const EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs)
{
	memset(SDObs->SdSatObs, 0, sizeof(SDObs->SdSatObs));
	SDObs->Time = EpkR->Time;
	SDObs->SatNum = 0;
	for (int i = 0; i < EpkR->SatNum; i++)
	{
		if (EpkR->SatPVT[i].Elevation * 180 / PI < 35)continue;
		if (EpkR->SatObs[i].cn0[0] < 30.0 || EpkR->SatObs[i].cn0[1] < 30.0)continue;
		if (EpkR->SatObs[i].LockTime[0] < 180 || EpkR->SatObs[i].LockTime[1] < 180)continue;
		if (EpkR->SatObs[i].Valid == false) continue;
		if (EpkR->SatPVT[i].Valid == false) continue;
		for (int j = 0; j < EpkB->SatNum; j++)
		{

			if (EpkR->SatObs[i].Prn == EpkB->SatObs[j].Prn && EpkR->SatObs[i].System == EpkB->SatObs[j].System)
			{
				if (EpkB->SatPVT[j].Elevation * 180 / PI < 40)continue;
				if (EpkB->SatObs[j].cn0[0] < 30.0 || EpkB->SatObs[j].cn0[1] < 30.0)continue;
				if (EpkB->SatObs[j].LockTime[0] < 180 || EpkB->SatObs[j].LockTime[1] < 180)continue;
				if (EpkB->SatObs[j].Valid == false) continue;
				if (EpkB->SatPVT[j].Valid == false) continue;
				SDObs->SdSatObs[SDObs->SatNum].Prn = EpkR->SatObs[i].Prn;
				SDObs->SdSatObs[SDObs->SatNum].System = EpkR->SatObs[i].System;
				SDObs->SdSatObs[SDObs->SatNum].dP[0] = EpkR->SatObs[i].P[0] - EpkB->SatObs[j].P[0] /*- EpkR->SatPVT[i].SatClkOft + EpkB->SatPVT[j].SatClkOft*/;
				SDObs->SdSatObs[SDObs->SatNum].dP[1] = EpkR->SatObs[i].P[1] - EpkB->SatObs[j].P[1]/* - EpkR->SatPVT[i].SatClkOft + EpkB->SatPVT[j].SatClkOft*/;
				SDObs->SdSatObs[SDObs->SatNum].dL[0] = EpkR->SatObs[i].L[0] - EpkB->SatObs[j].L[0] /*- EpkR->SatPVT[i].SatClkOft + EpkB->SatPVT[j].SatClkOft*/;
				SDObs->SdSatObs[SDObs->SatNum].dL[1] = EpkR->SatObs[i].L[1] - EpkB->SatObs[j].L[1] /*- EpkR->SatPVT[i].SatClkOft + EpkB->SatPVT[j].SatClkOft*/;
				SDObs->SdSatObs[SDObs->SatNum].iBas = j;
				SDObs->SdSatObs[SDObs->SatNum].iRov = i;
				SDObs->SdSatObs[SDObs->SatNum].Valid = true;
				SDObs->SatNum++;
				break;
			}
		}
	}
}


/**
* @brief 周跳探测函数
* @param SDObs 指向SDEPOCHOBS结构体的指针，包含单差观测数据
*/
void DetectCirclSlip(SDEPOCHOBS* SDObs)
{
	MWGF CurCObs[MAXCHANNUM];
	double f1 = 0.0, f2 = 0.0;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		CurCObs[i].Prn = SDObs->SdSatObs[i].Prn;
		CurCObs[i].Sys = SDObs->SdSatObs[i].System;
		if (SDObs->SdSatObs[i].Valid == false)continue;
		if (CurCObs[i].Sys == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; }
		else if (CurCObs[i].Sys == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
		CurCObs[i].MW = (SDObs->SdSatObs[i].dL[0] * f1 - SDObs->SdSatObs[i].dL[1] * f2) / (f1 - f2) - (SDObs->SdSatObs[i].dP[0] * f1 + SDObs->SdSatObs[i].dP[1] * f2) / (f1 + f2);
		CurCObs[i].GF = SDObs->SdSatObs[i].dL[0] - SDObs->SdSatObs[i].dL[1];
		CurCObs[i].n = 1;
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (CurCObs[i].Prn == SDObs->SdCObs[j].Prn && CurCObs[i].Sys == SDObs->SdCObs[j].Sys)    //如果当前卫星的组合观测
			{
				double dmw = fabs(CurCObs[i].MW - SDObs->SdCObs[j].MW);
				double dgf = fabs(CurCObs[i].GF - SDObs->SdCObs[j].GF);
				if(abs(dmw)<3.0&& abs(dgf)<0.05)    //如果组合观测值与已有的组合观测值差异小于阈值，则说明无粗差，更新组合观测值、平滑计数
				{
					CurCObs[i].n = SDObs->SdCObs[j].n + 1;
					CurCObs[i].MW = (CurCObs[i].MW * SDObs->SdCObs[j].n + SDObs->SdCObs[j].MW) / (SDObs->SdCObs[j].n + 1);
				}
				else SDObs->SdSatObs[i].Valid = false;
				break;
			}
		}
	}
	for(int i=0;i<MAXCHANNUM;i++)
	{
		SDObs->SdCObs[i] = CurCObs[i];
	}
}


/**
* @brief 参考星选择函数
* @param EpkB 基站历元观测数据指针
* @param EpkR 流动站历元观测数据指针
* @param SDObs 站间单差观测数据指针
* @param DDObs 双差观测数据指针
*/
void RefSatSelection(const EPOCHOBSDATA* EpkB, const EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	int i, n;
	double Sum[2] = { 0.0 }, MaxSum[2] = { 0.0 };
	int RefPrn[2] = { -1,-1 }, RefIndex[2] = { -1 };
	int GPSnum = 0, BDSnum = 0;
	/*基准星选取*/
	for (i = 0; i < SDObs->SatNum; i++)
	{
		if (!SDObs->SdSatObs[i].Valid || !EpkB->SatPVT[SDObs->SdSatObs[i].iBas].Valid || !EpkR->SatPVT[SDObs->SdSatObs[i].iRov].Valid) continue;
		n = SDObs->SdSatObs[i].System == GPS ? 0 : 1;
		if (n == 0) GPSnum++;
		else if (n == 1) BDSnum++;
		if (EpkB->SatObs[SDObs->SdSatObs[i].iBas].LockTime[0] < 6 || EpkB->SatObs[SDObs->SdSatObs[i].iBas].LockTime[1] < 6 ) continue;
		if (EpkR->SatObs[SDObs->SdSatObs[i].iRov].LockTime[0] < 6 || EpkR->SatObs[SDObs->SdSatObs[i].iRov].LockTime[1] < 6) continue;
		Sum[n] = EpkR->SatPVT[SDObs->SdSatObs[i].iRov].Elevation * 4*180 / PI + EpkB->SatObs[SDObs->SdSatObs[i].iBas].cn0[0] + EpkB->SatObs[SDObs->SdSatObs[i].iBas].cn0[1] +
			EpkR->SatObs[SDObs->SdSatObs[i].iRov].cn0[0] + EpkR->SatObs[SDObs->SdSatObs[i].iRov].cn0[1];
		if (Sum[n] > MaxSum[n])
		{
			MaxSum[n] = Sum[n];
			RefPrn[n] = SDObs->SdSatObs[i].Prn;
			RefIndex[n] = i;
		}
	}
	DDObs->DDSatNums[0] = (GPSnum == 0) ? GPSnum : GPSnum - 1;
	DDObs->DDSatNums[1] = (BDSnum == 0) ? BDSnum : BDSnum - 1;
	for (i = 0; i < 2; i++) 
	{
		DDObs->RefPrn[i] = RefPrn[i];
		if (MaxSum[i] < 200.0) DDObs->RefIndex[i] = -1;
		else DDObs->RefIndex[i] = RefIndex[i];
	}
}


/**
* @brief RTK主函数
* @param rawdata RTK原始数据结构体指针
* @param SDObs 站间单差观测数据结构体指针
* @param DDObs 双差观测数据结构体指针
* @param bestPos_Base 基站定位结果数据结构体指针
* @param SPPPos_rover 流动站单点定位结果数据结构体指针
*/
int RTK(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* SPPPos_rover, POSRES* RTKPos_Rover)
{
	double BasePos[3] = { 0.0 }, RoverPos[3 + (MAXCHANNUM - 2) * 2] = { 0.0 };
	if (abs(rawdata->bestPos.Pos[0]) < 0.001)return -1;
	for (int i = 0; i < 3; i++)
	{
		BasePos[i] = rawdata->bestPos.Pos[i];
		RoverPos[i] = SPPPos_rover->Pos[i];
	}
	int GPS_DDNum = 0, BDS_DDNum = 0;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0]) GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1]) BDS_DDNum++;
	}
	if (GPS_DDNum == 0 || BDS_DDNum == 0)if (GPS_DDNum + BDS_DDNum < 5)return -1;
	else if (GPS_DDNum > 0 && BDS_DDNum > 0)if (GPS_DDNum + BDS_DDNum < 3)return -1;
	double GPS_P_Coe = 1 / (2 * 0.09 * (GPS_DDNum + 1)), BDS_P_Coe = 1 / (2 * 0.09 * (BDS_DDNum + 1));
	double GPS_L_Coe = 1 / (2 * 0.0001 * (GPS_DDNum + 1)), BDS_L_Coe = 1 / (2 * 0.0001 * (BDS_DDNum + 1));
	double Base2Sat[MAXCHANNUM] = { 0.0 };
	for (int i = 0; i < rawdata->BaseObs.SatNum; i++)     //基站到卫星的距离
	{
		Base2Sat[i] = sqrt(pow(rawdata->BaseObs.SatPVT[i].SatPos[0] - BasePos[0], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[1] - BasePos[1], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[2] - BasePos[2], 2));
	}
	double w[(MAXCHANNUM - 2) * 2 * 2] = { 0.0 };
	double B[((MAXCHANNUM - 2) * 2 * 2) * (3 + (MAXCHANNUM - 2) * 2)] = { 0.0 };
	double P[((MAXCHANNUM - 2) * 2 * 2) * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double BT[(3 + (MAXCHANNUM - 2) * 2) * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 }, BTP[(3 + (MAXCHANNUM - 2) * 2) * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double BTPB[(3 + (MAXCHANNUM - 2) * 2) * (3 + (MAXCHANNUM - 2) * 2)] = { 0.0 }, INBTPB[(3 + (MAXCHANNUM - 2) * 2) * (3 + (MAXCHANNUM - 2) * 2)] = { 0.0 };
	double M[(3 + (MAXCHANNUM - 2) * 2) * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double X[3 + (MAXCHANNUM - 2) * 2] = { 0.0 };
	int iter = 0;
	double N_temp1 = 0.0, N_temp2 = 0.0;
	do {
		memset(w, 0, sizeof(w));
		memset(B, 0, sizeof(B));
		memset(P, 0, sizeof(P));
		double Rover2Ref[2] = { 0.0 };    //流动站到参考星的距离
		for (int i = 0; i < 2; i++)
		{
			if (!(DDObs->RefIndex[i] >= 0)) continue;
			int idx_R = SDObs->SdSatObs[DDObs->RefIndex[i]].iRov;
			Rover2Ref[i] = sqrt(pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[0] - RoverPos[0], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[1] - RoverPos[1], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[2] - RoverPos[2], 2));
		}
		int GPS_Tra = 0,BDS_Tra = 0;
		for (int i = 0; i < SDObs->SatNum; i++)
		{
			if (SDObs->SdSatObs[i].Valid == false) continue;
			if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
			if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
			if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
			int index = 0;
			double f1 = 0.0, f2 = 0.0;
			if (SDObs->SdSatObs[i].System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; index = 0; GPS_Tra++; }
			else if (SDObs->SdSatObs[i].System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; index = 1; BDS_Tra++; }
			double RefdP1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[0], RefdP2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[1];
			double RefdL1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[0], RefdL2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[1];
			int idx_bas = SDObs->SdSatObs[i].iBas;
			int idx_rov = SDObs->SdSatObs[i].iRov;

			//流动站到卫星的距离
			double Rover2Sat = sqrt(pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[0] - RoverPos[0], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[1] - RoverPos[1], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[2] - RoverPos[2], 2));
			double dP1 = SDObs->SdSatObs[i].dP[0];
			double dP2 = SDObs->SdSatObs[i].dP[1];
			double dL1 = SDObs->SdSatObs[i].dL[0];
			double dL2 = SDObs->SdSatObs[i].dL[1];
			double ddP1 = dP1 - RefdP1, ddP2 = dP2 - RefdP2, ddL1 = dL1 - RefdL1, ddL2 = dL2 - RefdL2;
			if (iter == 0) N_temp1 = (ddL1 - ddP1) * f1 / (C_Light), N_temp2 = (ddL2 - ddP2) * f2 / (C_Light);
			else
			{
				N_temp1 = (index == 0) ? RoverPos[3 + 2 * (GPS_Tra - 1)] : RoverPos[3 + 2 * GPS_DDNum + 2 * (BDS_Tra - 1)];
				N_temp2 = (index == 0) ? RoverPos[3 + 2 * (GPS_Tra - 1) + 1] : RoverPos[3 + 2 * GPS_DDNum + 2 * (BDS_Tra - 1) + 1];
			}
			double l = (RoverPos[0] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[0]) / Rover2Sat - (RoverPos[0] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[0]) / Rover2Ref[index];
			double m = (RoverPos[1] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[1]) / Rover2Sat - (RoverPos[1] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[1]) / Rover2Ref[index];
			double n = (RoverPos[2] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[2]) / Rover2Sat - (RoverPos[2] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[2]) / Rover2Ref[index];
			if ((index == 0 ? GPS_Tra == 0 : BDS_Tra == 0)) continue;
			RoverPos[3 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = N_temp1;
			RoverPos[3 + ((index == 0) ? 2 * (GPS_Tra - 1) + 1 : 2 * GPS_DDNum + 2 * (BDS_Tra - 1) + 1)] = N_temp2;
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 0] = ddP1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas]));
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1] = ddP2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas]));
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2] = ddL1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) - N_temp1 * C_Light / f1;
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3] = ddL2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) - N_temp2 * C_Light / f2;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * (3 + (MAXCHANNUM - 2) * 2) + 0] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * (3 + (MAXCHANNUM - 2) * 2) + 0] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * (3 + (MAXCHANNUM - 2) * 2) + 0] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * (3 + (MAXCHANNUM - 2) * 2) + 0] = l;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * (3 + (MAXCHANNUM - 2) * 2) + 1] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * (3 + (MAXCHANNUM - 2) * 2) + 1] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * (3 + (MAXCHANNUM - 2) * 2) + 1] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * (3 + (MAXCHANNUM - 2) * 2) + 1] = m;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * (3 + (MAXCHANNUM - 2) * 2) + 2] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * (3 + (MAXCHANNUM - 2) * 2) + 2] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * (3 + (MAXCHANNUM - 2) * 2) + 2] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * (3 + (MAXCHANNUM - 2) * 2) + 2] = n;
			B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * (3 + (MAXCHANNUM - 2) * 2) + 3 + (index == 0 ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f1;
			B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * (3 + (MAXCHANNUM - 2) * 2) + 4 + (index == 0 ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f2;
		}		
		for (int i = 0; i < 4 * (GPS_DDNum + BDS_DDNum); i++)
		{
			if (i < 4 * GPS_DDNum)
			{
				if (i % 4 == 0 || i % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = GPS_DDNum * GPS_P_Coe;
				else if (i % 4 == 2 || i % 4 == 3) P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = GPS_DDNum * GPS_L_Coe;
			}
			else
			{
				if (i % 4 == 0 || i % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = BDS_DDNum * BDS_P_Coe;
				else if (i % 4 == 2 || i % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = BDS_DDNum * BDS_L_Coe;
			}
			for (int j = 0; j < 4 * (GPS_DDNum + BDS_DDNum); j++)
			{
				if (i == j) continue;
				if (i < 4 * GPS_DDNum && j < 4 * GPS_DDNum)
				{
					if ((j - i) % 4 == 0)
					{
						if (j % 4 == 0 || j % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -GPS_P_Coe;
						else if (j % 4 == 2 || j % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -GPS_L_Coe;
					}
				}
				if (i > 4 * GPS_DDNum - 1 && j > 4 * GPS_DDNum - 1)
				{
					if ((j - i) % 4 == 0)
					{
						if (j % 4 == 0 || j % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -BDS_P_Coe;
						else if (j % 4 == 2 || j % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -BDS_L_Coe;
					}
				}
			}
		}
		MatrixTranspose(((MAXCHANNUM - 2) * 2 * 2), (3 + (MAXCHANNUM - 2) * 2), B, BT);
		MatrixMultiply((3 + (MAXCHANNUM - 2) * 2), ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), BT, P, BTP);
		MatrixMultiply((3 + (MAXCHANNUM - 2) * 2), ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), (3 + (MAXCHANNUM - 2) * 2), BTP, B, BTPB);
		if (!MatrixInverse((3 + (MAXCHANNUM - 2) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), BTPB, INBTPB))
		{
			cout << "矩阵求逆失败" << endl;
			return -1;
		}
		MatrixMultiply((3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), ((MAXCHANNUM - 2) * 2 * 2), INBTPB, BTP, M);
		MatrixMultiply((3 + (MAXCHANNUM - 2) * 2), ((MAXCHANNUM - 2) * 2 * 2), (MAXCHANNUM - 2) * 2 * 2, 1, M, w, X);
		for (int i = 0; i < 3 + (MAXCHANNUM - 2) * 2; i++)RoverPos[i] += X[i];
	} while ((++iter) < 10 && sqrt(pow(X[0], 2) + pow(X[1], 2) + pow(X[2], 2)) > 1e-4);
	double BX[((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 }, V[(3 + (MAXCHANNUM - 2) * 2)] = { 0.0 }, VT[(3 + (MAXCHANNUM - 2) * 2)] = { 0.0 }, VTV[1] = { 0 };
	MatrixMultiply(((MAXCHANNUM - 2) * 2 * 2), (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), 1, B, X, BX);
	MatrixSub((3 + (MAXCHANNUM - 2) * 2), 1, BX, w, V);
	MatrixTranspose((3 + (MAXCHANNUM - 2) * 2), 1, V, VT);
	MatrixMultiply(1, (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), VT, P, VT);
	MatrixMultiply(1, (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), 1, VT, V, VTV);
	RTKPos_Rover->SigmaPos = sqrt(VTV[0] / (2 * (GPS_DDNum + BDS_DDNum) - 3));
	RTKPos_Rover->PDOP = sqrt(INBTPB[0] + INBTPB[(3 + (MAXCHANNUM - 2) * 2) * 1 + 1] + INBTPB[(3 + (MAXCHANNUM - 2) * 2) * 2 + 2]);
	for (int i = 0; i < 3 + (MAXCHANNUM - 2) * 2; i++)
	{
		if (i < 3)RTKPos_Rover->Pos[i] = RoverPos[i];
		else DDObs->Float_Ambi[i - 3] = RoverPos[i];
	}
	memcpy(DDObs->Qnn, INBTPB, sizeof(double)* (3 + (MAXCHANNUM - 2) * 2)* (3 + (MAXCHANNUM - 2) * 2));
	return 0;
}


/**
* @param SPPPos_rover 流动站单点定位结果数据结构体指针
* @param EkfRTK 滤波器结构体指针
*/
void EKFINIT(POSRES* SPPPos_rover, EKFRTK* EkfRTK)
{
	memset(EkfRTK->Ambi, 0, sizeof(EkfRTK->Ambi));
	memset(EkfRTK->P, 0, sizeof(EkfRTK->P));
	memset(EkfRTK->X, 0, sizeof(EkfRTK->X));
	if (abs(SPPPos_rover->Pos[0]) < 0.5 || abs(SPPPos_rover->Pos[1]) < 0.5 || abs(SPPPos_rover->Pos[2]) < 0.5)return;
	int Pre_GPS_DDNum = 0, Pre_BDS_DDNum = 0;
	EkfRTK->Time = SPPPos_rover->Time;
	for (int i = 0; i < MAXGPSNUM; i++)
	{
		if (EkfRTK->DDSatPrn_GPS[i] == 0)break;
		if (i > 0)Pre_GPS_DDNum++;
	}
	for (int i = 0; i < MAXBDSNUM; i++)
	{
		if (EkfRTK->DDSatPrn_BDS[i] == 0)break;
		if (i > 0)Pre_BDS_DDNum++;
	}
	for (int i = 0; i < 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2; i++)
	{
		if (i < 3) EkfRTK->X[i] = SPPPos_rover->Pos[i], EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + i] = 25.0;
		else if (i >= 3 && i < 3 + Pre_GPS_DDNum * 2)
		{
			EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + i] = 9.0;
					continue;
		}
		
		else if (i >= Pre_GPS_DDNum * 2 + 3 && i < (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2 + 3)
		{
			
			EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + i] = 9.0;
			continue;	
		}
	}
	EkfRTK->isInited = true;
}


/**
* @brief 滤波时间预测函数
* @param rawdata RTK原始数据结构体指针
* @param SDObs 站间单差观测数据结构体指针
* @param DDObs 双差观测数据结构体指针
* @param RTKPos_Rover RTK定位结果结构体指针
* @param EkfRTK 滤波器结构体指针
*/
void TIMEPRED(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK)
{
	if (abs(rawdata->bestPos.Pos[0]) < 0.001)return ;
	int GPS_DDNum = 0, BDS_DDNum = 0;           
	int Pre_GPS_DDNum = 0, Pre_BDS_DDNum = 0; 
	int GPS_Tra = 0, BDS_Tra = 0;
	bool isRefChanged[2] = { false,false };   //当前历元参考星是否变化
	bool isRefExist[2] = { false,false };     //当前历元参考星是否存在于上个历元的双差卫星列表中
	int RefPos[2] = { -1,-1 };                //当前历元参考星在DDSatPrn中的位置
	int DDSatPrn_GPS[MAXGPSNUM] = { 0 };
	int DDSatPrn_BDS[MAXBDSNUM] = { 0 };
	double ambi[(MAXCHANNUM - 2) * 2] = { 0.0 };
	double* X, * P, * PreX, * PreP, * PHI, * Q;
	double* PHIT,*PHI_PreP,* PHIPPHIT;
	EkfRTK->Time = rawdata->RoverObs.Time;
	if (DDObs->RefPrn[0] != EkfRTK->DDSatPrn_GPS[0])isRefChanged[0] = true;
	if (DDObs->RefPrn[1] != EkfRTK->DDSatPrn_BDS[0])isRefChanged[1] = true;
	for (int i = 0; i < MAXGPSNUM; i++)
	{
		if (DDObs->RefPrn[0] == EkfRTK->DDSatPrn_GPS[i]) { isRefExist[0] = true; RefPos[0] = i; }
		if (EkfRTK->DDSatPrn_GPS[i] == 0)break;
		if (i > 0)Pre_GPS_DDNum++;
	}
	for (int i = 0; i < MAXBDSNUM; i++)
	{
		if (DDObs->RefPrn[1] == EkfRTK->DDSatPrn_BDS[i]) { isRefExist[1] = true; RefPos[1] = i; }
		if (EkfRTK->DDSatPrn_BDS[i] == 0)break;
		if (i > 0)Pre_BDS_DDNum++;
	}
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0])GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1])BDS_DDNum++;
	}
	X = new double[3 + (GPS_DDNum + BDS_DDNum) * 2] {0.0};
	P = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	PreX = new double[3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2] { 0.0 };
	PreP = new double[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * (3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2)] { 0.0 };
	PHI = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2)] {0.0};
	Q = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	PHIT = new double[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	PHI_PreP = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2)] {0.0};
	PHIPPHIT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	for (int i = 0; i < 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2; i++)
	{
		PreX[i] = EkfRTK->X[i];
		for (int j = 0; j < 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2; j++)PreP[i * (3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) + j] = EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j];
	}

	for (int i = 0; i < (BDS_DDNum + GPS_DDNum) * 2 + 3; i++)
	{
		if (i < 3) Q[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + i] = 1e-6;
		else Q[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + i] = 500;
	}
	for (int i = 0; i < 3; i++)PHI[i * (3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) + i] = 1.0;
	DDSatPrn_GPS[0] = DDObs->RefPrn[0];
	DDSatPrn_BDS[0] = DDObs->RefPrn[1];
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		bool isExist = false;
		int index = 0, pos = -1;
		double f1 = 0.0, f2 = 0.0;
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
		if (SDObs->SdSatObs[i].System == GPS)
		{
			f1 = FG1_GPS; 
			f2 = FG2_GPS; 
			index = 0; 
			GPS_Tra++; 
			DDSatPrn_GPS[GPS_Tra] = SDObs->SdSatObs[i].Prn;
		}
		else if (SDObs->SdSatObs[i].System == BDS) 
		{ 
			f1 = FG1_BDS;
			f2 = FG3_BDS; 
			index = 1; 
			BDS_Tra++;
			DDSatPrn_BDS[BDS_Tra] = SDObs->SdSatObs[i].Prn;
		}
		double RefdP1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[0], RefdP2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[1];
		double RefdL1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[0], RefdL2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[1];
		double dP1 = SDObs->SdSatObs[i].dP[0];
		double dP2 = SDObs->SdSatObs[i].dP[1];
		double dL1 = SDObs->SdSatObs[i].dL[0];
		double dL2 = SDObs->SdSatObs[i].dL[1];
		double ddP1 = dP1 - RefdP1, ddP2 = dP2 - RefdP2, ddL1 = dL1 - RefdL1, ddL2 = dL2 - RefdL2;
		double N_temp1 = (ddL1 - ddP1) * f1 / (C_Light), N_temp2 = (ddL2 - ddP2) * f2 / (C_Light);
		for (int j = 0; j < ((index == 0) ? MAXGPSNUM : MAXBDSNUM); j++)
		{
			if (SDObs->SdSatObs[i].Prn == ((index == 0) ? EkfRTK->DDSatPrn_GPS[j] : EkfRTK->DDSatPrn_BDS[j])) { isExist = true; pos = j; break; }
			if (((index == 0) ? EkfRTK->DDSatPrn_GPS[j] : EkfRTK->DDSatPrn_BDS[j]) == 0)break;
		}
		ambi[(index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1)] = N_temp1;
		ambi[(index == 0) ? 2 * (GPS_Tra - 1) + 1 : 2 * GPS_DDNum + 2 * (BDS_Tra - 1) + 1] = N_temp2;
		if (isRefChanged[index])   //参考星改变
		{
			if (isRefExist[index] && isExist && pos == 0)
			{
				//Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1))] = 1;
				//Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2))] = 1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + 3 + ((index == 0) ? (RefPos[0] - 1) * 2 : (Pre_GPS_DDNum + RefPos[1] - 1) * 2)] = -1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + 4 + ((index == 0) ? (RefPos[0] - 1) * 2 : (Pre_GPS_DDNum + RefPos[1] - 1) * 2)] = -1;
			}
			else if (isRefExist[index] && isExist && pos != 0)
			{
				//Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1))] = 1;
				//Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2))] = 1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + 3 + ((index == 0) ? (pos - 1) * 2 : (Pre_GPS_DDNum + pos - 1) * 2)] = 1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + 3 + ((index == 0) ? (RefPos[0] - 1) * 2 : (Pre_GPS_DDNum + RefPos[1] - 1) * 2)] = -1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + 4 + ((index == 0) ? (pos - 1) * 2 : (Pre_GPS_DDNum + pos - 1) * 2)] = 1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + 4 + ((index == 0) ? (RefPos[0] - 1) * 2 : (Pre_GPS_DDNum + RefPos[1] - 1) * 2)] = -1;
			}
		}
		else   //参考星未改变
		{
			if (isExist)
			{
				Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1))] = 1e-10;
				Q[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2))] = 1e-10;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 1) : (2 * (GPS_DDNum + BDS_Tra) + 1)) + 3 + ((index == 0) ? (pos - 1) * 2 : (Pre_GPS_DDNum + pos - 1) * 2)] = 1;
				PHI[(3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2) * ((index == 0) ? (2 * GPS_Tra + 2) : (2 * (GPS_DDNum + BDS_Tra) + 2)) + 4 + ((index == 0) ? (pos - 1) * 2 : (Pre_GPS_DDNum + pos - 1) * 2)] = 1;
			}
		}
	}
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 1, PHI, PreX, X);
	for (int i = 0; i < 2 * (GPS_DDNum + BDS_DDNum); i++) if (abs(X[3 + i]) < 1e-8)X[3 + i] = ambi[i];
	MatrixTranspose(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, PHI, PHIT);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, PHI, PreP, PHI_PreP);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 3 + (Pre_GPS_DDNum + Pre_BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, PHI_PreP, PHIT, PHIPPHIT);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, PHIPPHIT, Q, P);
	memset(EkfRTK->X, 0, sizeof(EkfRTK->X));
	memset(EkfRTK->P, 0, sizeof(EkfRTK->P));
	memset(EkfRTK->Ambi, 0, sizeof(EkfRTK->Ambi));
	memset(EkfRTK->DDSatPrn_GPS, 0, sizeof(EkfRTK->DDSatPrn_GPS));
	memset(EkfRTK->DDSatPrn_BDS, 0, sizeof(EkfRTK->DDSatPrn_BDS));
	memcpy(EkfRTK->DDSatPrn_GPS, DDSatPrn_GPS, sizeof(DDSatPrn_GPS));
	memcpy(EkfRTK->DDSatPrn_BDS, DDSatPrn_BDS, sizeof(DDSatPrn_BDS));
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		EkfRTK->X[i] = X[i];
		if (i < 3)RTKPos_Rover->Pos[i] = X[i];
		if (i >= 3)EkfRTK->Ambi[i - 3] = X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j] = P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j];
	}
	delete[] X; delete[]P; delete[]PreX, delete[]PreP; delete[]PHI; delete[]Q;
	delete[] PHIT; delete[]PHI_PreP; delete[]PHIPPHIT;
}


/**
* @brief 滤波观测更新函数
* @param rawdata RTK原始数据结构体指针
* @param SDObs 站间单差观测数据结构体指针
* @param DDObs 双差观测数据结构体指针
* @param RTKPos_Rover RTK定位结果结构体指针
* @param EkfRTK 滤波器结构体指针
*/
void EKFUPDATE(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK)
{
	int GPS_DDNum = 0, BDS_DDNum = 0;
	int GPS_Tra = 0, BDS_Tra = 0;
	double Base2Sat[MAXCHANNUM] = { 0.0 };
	double Rover2Ref[2] = { 0.0 };
	double* X, * P, * I, * Z, * H, * FX, * R;
	double* HT, * PHT, * HPHT, * HPHTR, * INHPHTR, * K, * ZFX, * KZFX, * KT, * KH, * IKH, * IKHT, * IKHP, * KR, * KRKT;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0])GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1])BDS_DDNum++;
	}
	X = new double[3 + (GPS_DDNum + BDS_DDNum) * 2] {0.0};
	P = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	Z = new double[(GPS_DDNum + BDS_DDNum) * 2 * 2] {0.0};
	I = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	H = new double[(GPS_DDNum + BDS_DDNum) * 2 * 2 * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	FX = new double[(GPS_DDNum + BDS_DDNum) * 2 * 2] {0.0};
	R = new double[(GPS_DDNum + BDS_DDNum) * 4 * (GPS_DDNum + BDS_DDNum) * 4] {0.0};
	HT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	PHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	HPHT = new double[((GPS_DDNum + BDS_DDNum) * 2 * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	HPHTR = new double[((GPS_DDNum + BDS_DDNum) * 2 * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	INHPHTR = new double[((GPS_DDNum + BDS_DDNum) * 2 * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	K = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (GPS_DDNum + BDS_DDNum) * 4] {0.0};
	ZFX = new double[(GPS_DDNum + BDS_DDNum) * 2 * 2] {0.0};
	KZFX = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KT = new double[((GPS_DDNum + BDS_DDNum) * 2 * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	IKHP = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KR = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2 * 2)] {0.0};
	KRKT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};

	for (int i = 0; i < rawdata->BaseObs.SatNum; i++)
	{
		Base2Sat[i] = sqrt(pow(rawdata->BaseObs.SatPVT[i].SatPos[0] - rawdata->bestPos.Pos[0], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[1] - rawdata->bestPos.Pos[1], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[2] - rawdata->bestPos.Pos[2], 2));
	}
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		I[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + i] = 1.0;
		X[i] = EkfRTK->X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j] = EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j];
	}
	for (int i = 0; i < 2; i++)
	{
		if (!(DDObs->RefIndex[i] >= 0)) continue;
		int idx_R = SDObs->SdSatObs[DDObs->RefIndex[i]].iRov;
		Rover2Ref[i] = sqrt(pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[0] - X[0], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[1] - X[1], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[2] - X[2], 2));
	}
	
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		int index = 0;
		double f1 = 0.0, f2 = 0.0;
		if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
		if (SDObs->SdSatObs[i].System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; index = 0; GPS_Tra++; }
		else if (SDObs->SdSatObs[i].System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; index = 1; BDS_Tra++; }
		double RefdP1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[0], RefdP2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[1];
		double RefdL1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[0], RefdL2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[1];
		double dP1 = SDObs->SdSatObs[i].dP[0];
		double dP2 = SDObs->SdSatObs[i].dP[1];
		double dL1 = SDObs->SdSatObs[i].dL[0];
		double dL2 = SDObs->SdSatObs[i].dL[1];
		double ddP1 = dP1 - RefdP1, ddP2 = dP2 - RefdP2, ddL1 = dL1 - RefdL1, ddL2 = dL2 - RefdL2;
		double N_temp1 = (ddL1 - ddP1) * f1 / (C_Light), N_temp2 = (ddL2 - ddP2) * f2 / (C_Light);
		int idx_bas = SDObs->SdSatObs[i].iBas;
		int idx_rov = SDObs->SdSatObs[i].iRov;
		double Rover2Sat = sqrt(pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[0] - X[0], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[1] - X[1], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[2] - X[2], 2));
		double l = (X[0] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[0]) / Rover2Sat - (X[0] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[0]) / Rover2Ref[index];
		double m = (X[1] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[1]) / Rover2Sat - (X[1] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[1]) / Rover2Ref[index];
		double n = (X[2] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[2]) / Rover2Sat - (X[2] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[2]) / Rover2Ref[index];
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 0 : 4 * (GPS_DDNum + BDS_Tra - 1) + 0)] = ddP1;
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 1 : 4 * (GPS_DDNum + BDS_Tra - 1) + 1)] = ddP2;
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 2 : 4 * (GPS_DDNum + BDS_Tra - 1) + 2)] = ddL1;
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 3 : 4 * (GPS_DDNum + BDS_Tra - 1) + 3)] = ddL2;
		FX[((index == 0) ? (GPS_Tra - 1) * 4 + 0 : (BDS_Tra - 1) * 4 + 0 + GPS_DDNum * 4)] = FX[((index == 0) ? (GPS_Tra - 1) * 4 + 1 : (BDS_Tra - 1) * 4 + 1 + GPS_DDNum * 4)] = Rover2Sat - Base2Sat[idx_bas] - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas]);
		FX[((index == 0) ? (GPS_Tra - 1) * 4 + 2 : (BDS_Tra - 1) * 4 + 2 + GPS_DDNum * 4)] = FX[((index == 0) ? (GPS_Tra - 1) * 4 + 0 : (BDS_Tra - 1) * 4 + 0 + GPS_DDNum * 4)] + C_Light * X[((index == 0) ? (GPS_Tra - 1) * 2 + 3 : (BDS_Tra - 1) * 2 + 3 + GPS_DDNum * 2)] / f1;
		FX[((index == 0) ? (GPS_Tra - 1) * 4 + 3 : (BDS_Tra - 1) * 4 + 3 + GPS_DDNum * 4)] = FX[((index == 0) ? (GPS_Tra - 1) * 4 + 1 : (BDS_Tra - 1) * 4 + 1 + GPS_DDNum * 4)] + C_Light * X[((index == 0) ? (GPS_Tra - 1) * 2 + 4 : (BDS_Tra - 1) * 2 + 4 + GPS_DDNum * 2)] / f2;
		//Z[((index == 0) ? 4 * (GPS_Tra - 1) + 0 : 4 * (GPS_DDNum + BDS_Tra - 1) + 0)] = ddP1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		//Z[((index == 0) ? 4 * (GPS_Tra - 1) + 1 : 4 * (GPS_DDNum + BDS_Tra - 1) + 1)] = ddP2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		//Z[((index == 0) ? 4 * (GPS_Tra - 1) + 2 : 4 * (GPS_DDNum + BDS_Tra - 1) + 2)] = ddL1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		//Z[((index == 0) ? 4 * (GPS_Tra - 1) + 3 : 4 * (GPS_DDNum + BDS_Tra - 1) + 3)] = ddL2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];

		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 0] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 0] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 0] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 0] = l;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 1] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 1] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 1] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 1] = m;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 2] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 2] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 2] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 2] = n;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 3 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f1;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 4 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f2;
	}
	for (int i = 0; i < 4 * (GPS_DDNum + BDS_DDNum); i++)
	{
		if (i % 4 == 0 || i % 4 == 1)R[((GPS_DDNum + BDS_DDNum) * 2 * 2) * i + i] = 0.09;
		else if (i % 4 == 2 || i % 4 == 3) R[((GPS_DDNum + BDS_DDNum) * 2 * 2) * i + i] = 0.0001;
		for (int j = 0; j < 4 * (GPS_DDNum + BDS_DDNum); j++)
		{
			if (i == j) continue;
			if ((j - i) % 4 == 0 && ((j < GPS_Tra * 4 && i < GPS_Tra * 4) || (j >= GPS_Tra * 4 && i >= GPS_Tra * 4)))
			{
				if (j % 4 == 0 || j % 4 == 1)R[((GPS_DDNum + BDS_DDNum) * 2 * 2) * i + j] = 0.5*0.09;
				else if (j % 4 == 2 || j % 4 == 3)R[((GPS_DDNum + BDS_DDNum) * 2 * 2) * i + j] = 0.5*0.0001;
			}
		}
	}
	MatrixTranspose(((GPS_DDNum + BDS_DDNum) * 2 * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), H, HT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), P, HT, PHT);
	MatrixMultiply(((GPS_DDNum + BDS_DDNum) * 2 * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), H, PHT, HPHT);
	MatrixAdd(((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), HPHT, R, HPHTR);
	MatrixInverse(((GPS_DDNum + BDS_DDNum) * 2 * 2), (GPS_DDNum + BDS_DDNum) * 2 * 2, HPHTR, INHPHTR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), (GPS_DDNum + BDS_DDNum) * 2 * 2, PHT, INHPHTR, K);
	//MatrixMultiply((GPS_DDNum + BDS_DDNum) * 2 * 2, (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), 1, H, X, FX);
	MatrixSub((GPS_DDNum + BDS_DDNum) * 2 * 2, 1, Z, FX, ZFX);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (GPS_DDNum + BDS_DDNum) * 2 * 2, (GPS_DDNum + BDS_DDNum) * 2 * 2, 1, K, ZFX, KZFX);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 1, X, KZFX, X);
	MatrixTranspose((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), K, KT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), K, H, KH);
	MatrixSub(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, I, KH, IKH);
	MatrixTranspose(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, IKHT);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, P, IKHP);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKHP, IKHT, P);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), K, R, KR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), KR, KT, KRKT);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, P, KRKT, P);
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		EkfRTK->X[i] = X[i];
		if (i < 3)RTKPos_Rover->Pos[i] = X[i];
		if (i >= 3)EkfRTK->Ambi[i - 3] = X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j] = P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j];
	}
	RTKPos_Rover->PDOP = sqrt(P[0] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 1 + 1] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 2 + 2]);
	delete[]X; delete[] P; delete[] I; delete[] Z; delete[]H; delete[]FX; delete[] R;
	delete[]HT; delete[]PHT; delete[]HPHT; delete[] HPHTR; delete[]INHPHTR; delete[] K;  delete[]ZFX;  delete[]KZFX; delete[]KT; delete[] KH; delete[]IKH; delete[]IKHT; delete[]IKHP; delete[]KR; delete[]KRKT;
}


/**
* @brief 固定解函数
* @param rawdata RTK原始数据结构体指针
* @param SDObs 站间单差观测数据结构体指针
* @param DDObs 双差观测数据结构体指针
* @param RTKPos_Rover RTK定位结果结构体指针
* @param Fixed 固定的模糊度
*/
void RTK_FIXED(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, double Fixed[])
{
	double BasePos[3] = { 0.0 }, RoverPos[3] = { 0.0 };
	if (abs(rawdata->bestPos.Pos[0]) < 0.001)return;
	for (int i = 0; i < 3; i++)
	{
		BasePos[i] = rawdata->bestPos.Pos[i];
		RoverPos[i] = RTKPos_Rover->Pos[i];
	}
	int GPS_DDNum = 0, BDS_DDNum = 0;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0]) GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1]) BDS_DDNum++;
	}
	double GPS_P_Coe = 1 / (2 * 0.09 * (GPS_DDNum + 1)), BDS_P_Coe = 1 / (2 * 0.09 * (BDS_DDNum + 1));
	double GPS_L_Coe = 1 / (2 * 0.0001 * (GPS_DDNum + 1)), BDS_L_Coe = 1 / (2 * 0.0001 * (BDS_DDNum + 1));
	double Base2Sat[MAXCHANNUM] = { 0.0 };
	for (int i = 0; i < rawdata->BaseObs.SatNum; i++)
	{
		Base2Sat[i] = sqrt(pow(rawdata->BaseObs.SatPVT[i].SatPos[0] - BasePos[0], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[1] - BasePos[1], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[2] - BasePos[2], 2));
	}
	double w[(MAXCHANNUM - 2) * 2 * 2] = { 0.0 };
	double B[((MAXCHANNUM - 2) * 2 * 2) * 3] = { 0.0 };
	double P[((MAXCHANNUM - 2) * 2 * 2) * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double BT[3 * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 }, BTP[3 * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double BTPB[3 * 3] = { 0.0 }, INBTPB[3 * 3] = { 0.0 };
	double M[3 * ((MAXCHANNUM - 2) * 2 * 2)] = { 0.0 };
	double X[3] = { 0.0 };
	int iter = 0;
	double N_temp1 = 0.0, N_temp2 = 0.0;
	do {
		memset(w, 0, sizeof(w));
		memset(B, 0, sizeof(B));
		memset(P, 0, sizeof(P));
		double Rover2Ref[2] = { 0.0 };
		for (int i = 0; i < 2; i++)
		{
			if (!(DDObs->RefIndex[i] >= 0)) continue;
			int idx_R = SDObs->SdSatObs[DDObs->RefIndex[i]].iRov;
			Rover2Ref[i] = sqrt(pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[0] - RoverPos[0], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[1] - RoverPos[1], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[2] - RoverPos[2], 2));
		}
		int GPS_Tra = 0, BDS_Tra = 0;
		for (int i = 0; i < SDObs->SatNum; i++)
		{
			if (SDObs->SdSatObs[i].Valid == false) continue;
			if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
			if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
			if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
			int index = 0;
			double f1 = 0.0, f2 = 0.0;
			if (SDObs->SdSatObs[i].System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; index = 0; GPS_Tra++; }
			else if (SDObs->SdSatObs[i].System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; index = 1; BDS_Tra++; }
			double RefdP1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[0], RefdP2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[1];
			double RefdL1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[0], RefdL2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[1];
			int idx_bas = SDObs->SdSatObs[i].iBas;
			int idx_rov = SDObs->SdSatObs[i].iRov;
			double Rover2Sat = sqrt(pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[0] - RoverPos[0], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[1] - RoverPos[1], 2) +
				pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[2] - RoverPos[2], 2));
			double dP1 = SDObs->SdSatObs[i].dP[0];
			double dP2 = SDObs->SdSatObs[i].dP[1];
			double dL1 = SDObs->SdSatObs[i].dL[0];
			double dL2 = SDObs->SdSatObs[i].dL[1];
			double ddP1 = dP1 - RefdP1, ddP2 = dP2 - RefdP2, ddL1 = dL1 - RefdL1, ddL2 = dL2 - RefdL2;
			double l = (RoverPos[0] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[0]) / Rover2Sat - (RoverPos[0] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[0]) / Rover2Ref[index];
			double m = (RoverPos[1] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[1]) / Rover2Sat - (RoverPos[1] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[1]) / Rover2Ref[index];
			double n = (RoverPos[2] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[2]) / Rover2Sat - (RoverPos[2] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[2]) / Rover2Ref[index];
			if ((index == 0 ? GPS_Tra == 0 : BDS_Tra == 0)) continue;
			N_temp1 = Fixed[(index == 0 ? (GPS_Tra - 1) * 2 : (GPS_DDNum + BDS_Tra - 1) * 2)], N_temp2 = Fixed[(index == 0 ? (GPS_Tra - 1) * 2 + 1 : (GPS_DDNum + BDS_Tra - 1) * 2 + 1)];
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 0] = ddP1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas]));
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1] = ddP2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas]));
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2] = ddL1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) - N_temp1 * C_Light / f1;
			w[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3] = ddL2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) - N_temp2 * C_Light / f2;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * 3 + 0] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * 3 + 0] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * 3 + 0] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * 3 + 0] = l;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * 3 + 1] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * 3 + 1] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * 3 + 1] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * 3 + 1] = m;
			B[(index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) * 3 + 2] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 1) * 3 + 2] =
				B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 2) * 3 + 2] = B[((index == 0 ? (GPS_Tra - 1) * 4 : 4 * GPS_DDNum + (BDS_Tra - 1) * 4) + 3) * 3 + 2] = n;
		}
		for (int i = 0; i < 4 * (GPS_DDNum + BDS_DDNum); i++)
		{
			if (i < 4 * GPS_DDNum)
			{
				if (i % 4 == 0 || i % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = GPS_DDNum * GPS_P_Coe;
				else if (i % 4 == 2 || i % 4 == 3) P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = GPS_DDNum * GPS_L_Coe;
			}
			else
			{
				if (i % 4 == 0 || i % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = BDS_DDNum * BDS_P_Coe;
				else if (i % 4 == 2 || i % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + i] = BDS_DDNum * BDS_L_Coe;
			}
			for (int j = 0; j < 4 * (GPS_DDNum + BDS_DDNum); j++)
			{
				if (i == j) continue;
				if (i < 4 * GPS_DDNum && j < 4 * GPS_DDNum)
				{
					if ((j - i) % 4 == 0)
					{
						if (j % 4 == 0 || j % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -GPS_P_Coe;
						else if (j % 4 == 2 || j % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -GPS_L_Coe;
					}
				}
				if (i > 4 * GPS_DDNum - 1 && j > 4 * GPS_DDNum - 1)
				{
					if ((j - i) % 4 == 0)
					{
						if (j % 4 == 0 || j % 4 == 1)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -BDS_P_Coe;
						else if (j % 4 == 2 || j % 4 == 3)P[((MAXCHANNUM - 2) * 2 * 2) * i + j] = -BDS_L_Coe;
					}
				}
			}
		}
		MatrixTranspose(((MAXCHANNUM - 2) * 2 * 2), 3, B, BT);
		MatrixMultiply(3, ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), BT, P, BTP);
		MatrixMultiply(3, ((MAXCHANNUM - 2) * 2 * 2), ((MAXCHANNUM - 2) * 2 * 2), 3, BTP, B, BTPB);
		if (!MatrixInverse(3, 3, BTPB, INBTPB))
		{
			cout << "矩阵求逆失败" << endl;
			return ;
		}
		MatrixMultiply(3, 3, 3, ((MAXCHANNUM - 2) * 2 * 2), INBTPB, BTP, M);
		MatrixMultiply(3, ((MAXCHANNUM - 2) * 2 * 2), (MAXCHANNUM - 2) * 2 * 2, 1, M, w, X);
		for (int i = 0; i < 3; i++)
		{
			RoverPos[i] += X[i];
		}
	} while ((++iter) < 10 && sqrt(pow(X[0], 2) + pow(X[1], 2) + pow(X[2], 2)) > 1e-4);
	double BX[((MAXCHANNUM - 2) * 2 * 2)] = { 0 }, V[(3 + (MAXCHANNUM - 2) * 2)] = { 0 }, VT[(3 + (MAXCHANNUM - 2) * 2)] = { 0 }, VTV[1] = { 0 };
	MatrixMultiply(((MAXCHANNUM - 2) * 2 * 2), 3, 3, 1, B, X, BX);
	MatrixSub((3 + (MAXCHANNUM - 2) * 2), 1, BX, w, V);
	MatrixTranspose((3 + (MAXCHANNUM - 2) * 2), 1, V, VT);
	MatrixMultiply(1, (3 + (MAXCHANNUM - 2) * 2), (3 + (MAXCHANNUM - 2) * 2), 1, VT, V, VTV);
	RTKPos_Rover->SigmaPos = sqrt(VTV[0] / (4 * (GPS_DDNum + BDS_DDNum) - 3));
	RTKPos_Rover->PDOP = sqrt(INBTPB[0] + INBTPB[4] + INBTPB[8]);
	for (int i = 0; i < 3; i++)RTKPos_Rover->Pos[i] = RoverPos[i];
}

/**
* @brief 滤波固定解函数
* @param rawdata RTK原始数据结构体指针
* @param SDObs 站间单差观测数据结构体指针
* @param DDObs 双差观测数据结构体指针
* @param RTKPos_Rover RTK定位结果结构体指针
* @param EkfRTK 滤波器结构体指针
* @param Fixed 固定的模糊度
*/
void KF_FIXED1(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK, double Fixed[])
{
	int GPS_DDNum = 0, BDS_DDNum = 0;
	int GPS_Tra = 0, BDS_Tra = 0;
	double Base2Sat[MAXCHANNUM] = { 0.0 };
	double Rover2Ref[2] = { 0.0 };
	double* X, * P, * I, * Z, * H, * FX, * R;
	double* HT, * PHT, * HPHT, * HPHTR, * INHPHTR, * K, * ZFX, * KZFX, * KT, * KH, * IKH, * IKHT, * IKHP, * KR, * KRKT;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0])GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1])BDS_DDNum++;
	}
	X = new double[3 + (GPS_DDNum + BDS_DDNum) * 2] {0.0};
	P = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	Z = new double[(GPS_DDNum + BDS_DDNum) * 6] {0.0};
	I = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	H = new double[((GPS_DDNum + BDS_DDNum) * 6) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	FX = new double[(GPS_DDNum + BDS_DDNum) * 6] {0.0};
	R = new double[((GPS_DDNum + BDS_DDNum) * 6) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	HT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	PHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	HPHT = new double[((GPS_DDNum + BDS_DDNum) * 6) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	HPHTR = new double[((GPS_DDNum + BDS_DDNum) * 6) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	INHPHTR = new double[((GPS_DDNum + BDS_DDNum) * 6) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	K = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	ZFX = new double[(GPS_DDNum + BDS_DDNum) * 6] {0.0};
	KZFX = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KT = new double[((GPS_DDNum + BDS_DDNum) * 6) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKHP = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KR = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 6)] {0.0};
	KRKT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};

	for (int i = 0; i < rawdata->BaseObs.SatNum; i++)
	{
		Base2Sat[i] = sqrt(pow(rawdata->BaseObs.SatPVT[i].SatPos[0] - rawdata->bestPos.Pos[0], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[1] - rawdata->bestPos.Pos[1], 2) +
			pow(rawdata->BaseObs.SatPVT[i].SatPos[2] - rawdata->bestPos.Pos[2], 2));
	}
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		I[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + i] = 1.0;
		X[i] = EkfRTK->X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j] = EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j];
	}
	for (int i = 0; i < 2; i++)
	{
		if (!(DDObs->RefIndex[i] >= 0)) continue;
		int idx_R = SDObs->SdSatObs[DDObs->RefIndex[i]].iRov;
		Rover2Ref[i] = sqrt(pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[0] - X[0], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[1] - X[1], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_R].SatPos[2] - X[2], 2));
	}

	for (int i = 0; i < SDObs->SatNum; i++)
	{
		int index = 0;
		double f1 = 0.0, f2 = 0.0;
		if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
		if (SDObs->SdSatObs[i].System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; index = 0; GPS_Tra++; }
		else if (SDObs->SdSatObs[i].System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; index = 1; BDS_Tra++; }
		double RefdP1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[0], RefdP2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dP[1];
		double RefdL1 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[0], RefdL2 = SDObs->SdSatObs[DDObs->RefIndex[index]].dL[1];
		double dP1 = SDObs->SdSatObs[i].dP[0];
		double dP2 = SDObs->SdSatObs[i].dP[1];
		double dL1 = SDObs->SdSatObs[i].dL[0];
		double dL2 = SDObs->SdSatObs[i].dL[1];
		double ddP1 = dP1 - RefdP1, ddP2 = dP2 - RefdP2, ddL1 = dL1 - RefdL1, ddL2 = dL2 - RefdL2;
		double N_temp1 = (ddL1 - ddP1) * f1 / (C_Light), N_temp2 = (ddL2 - ddP2) * f2 / (C_Light);
		int idx_bas = SDObs->SdSatObs[i].iBas;
		int idx_rov = SDObs->SdSatObs[i].iRov;
		double Rover2Sat = sqrt(pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[0] - X[0], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[1] - X[1], 2) +
			pow(rawdata->RoverObs.SatPVT[idx_rov].SatPos[2] - X[2], 2));
		double l = (X[0] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[0]) / Rover2Sat - (X[0] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[0]) / Rover2Ref[index];
		double m = (X[1] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[1]) / Rover2Sat - (X[1] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[1]) / Rover2Ref[index];
		double n = (X[2] - rawdata->RoverObs.SatPVT[idx_rov].SatPos[2]) / Rover2Sat - (X[2] - rawdata->RoverObs.SatPVT[SDObs->SdSatObs[DDObs->RefIndex[index]].iRov].SatPos[2]) / Rover2Ref[index];
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 0 : 4 * (GPS_DDNum + BDS_Tra - 1) + 0)] = ddP1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 1 : 4 * (GPS_DDNum + BDS_Tra - 1) + 1)] = ddP2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 2 : 4 * (GPS_DDNum + BDS_Tra - 1) + 2)] = ddL1 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		Z[((index == 0) ? 4 * (GPS_Tra - 1) + 3 : 4 * (GPS_DDNum + BDS_Tra - 1) + 3)] = ddL2 - ((Rover2Sat - Base2Sat[idx_bas]) - (Rover2Ref[index] - Base2Sat[SDObs->SdSatObs[DDObs->RefIndex[index]].iBas])) + l * X[0] + m * X[1] + n * X[2];
		Z[((index == 0) ? 4 * (GPS_DDNum + BDS_DDNum) + (GPS_Tra - 1) * 2 : 4 * (GPS_DDNum + BDS_DDNum) + (GPS_DDNum + BDS_Tra - 1) * 2)] = Fixed[(index == 0) ? (GPS_Tra - 1) * 2 : (GPS_DDNum + BDS_Tra - 1) * 2];
		Z[((index == 0) ? 4 * (GPS_DDNum + BDS_DDNum) + (GPS_Tra - 1) * 2 + 1 : 4 * (GPS_DDNum + BDS_DDNum) + (GPS_DDNum + BDS_Tra - 1) * 2 + 1)] = Fixed[(index == 0) ? (GPS_Tra - 1) * 2 + 1 : (GPS_DDNum + BDS_Tra - 1) * 2 + 1];
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 0] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 0] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 0] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 0] = l;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 1] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 1] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 1] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 1] = m;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 0) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 0)) + 2] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 1) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 1)) + 2] =
			H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 2] = H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 2] = n;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 2) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 2)) + 3 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f1;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? ((GPS_Tra - 1) * 4 + 3) : ((4 * GPS_DDNum) + (BDS_Tra - 1) * 4 + 3)) + 4 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = C_Light / f2;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? 4 * (GPS_DDNum + BDS_DDNum) + 2 * (GPS_Tra - 1) : 4 * (GPS_DDNum + BDS_DDNum) + 2 * (GPS_DDNum + BDS_Tra - 1)) + 3 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = 1.0;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? 4 * (GPS_DDNum + BDS_DDNum) + 2 * (GPS_Tra - 1) + 1 : 4 * (GPS_DDNum + BDS_DDNum) + 2 * (GPS_DDNum + BDS_Tra - 1) + 1) + 4 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = 1.0;
	}
	for (int i = 0; i < (GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		if (i < (GPS_DDNum + BDS_DDNum) * 2 * 2)
		{
			if (i % 4 == 0 || i % 4 == 1)R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + i] = 0.09;
			else if (i % 4 == 2 || i % 4 == 3) R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + i] = 0.0001;
			for (int j = 0; j < 4 * (GPS_DDNum + BDS_DDNum); j++)
			{
				if (i == j) continue;
				if ((j - i) % 4 == 0 && ((j < GPS_Tra * 4 && i < GPS_Tra * 4) || (j >= GPS_Tra * 4 && i >= GPS_Tra * 4)))
				{
					if (j % 4 == 0 || j % 4 == 1)R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + j] = 0.5 * 0.09;
					else if (j % 4 == 2 || j % 4 == 3)R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + j] = 0.5 * 0.0001;
				}
			}
		}
		else
		{
			R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + i] = 1e-10;
			for(int j = 4 * (GPS_DDNum + BDS_DDNum); j < (GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2; j++)
			{
				if (i == j) continue;
				if ((j - i) % 2 == 0)
				{
					if (j < (GPS_DDNum + BDS_DDNum) * 4 + 2 * GPS_DDNum && i< (GPS_DDNum + BDS_DDNum) * 4 + 2 * GPS_DDNum)R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + j] = 0.5 * 1e-10;
					else if(j > (GPS_DDNum + BDS_DDNum) * 4 + 2 * GPS_DDNum && i > (GPS_DDNum + BDS_DDNum) * 4 + 2 * GPS_DDNum)R[((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2) * i + j] = 0.5 * 1e-10;
				}
				
			}
		}
	}
	MatrixTranspose((GPS_DDNum + BDS_DDNum) * 6, 3 + (GPS_DDNum + BDS_DDNum) * 2, H, HT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), P, HT, PHT);
	MatrixMultiply(((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2), 3 + (GPS_DDNum + BDS_DDNum) * 2, (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), H, PHT, HPHT);
	MatrixAdd((GPS_DDNum + BDS_DDNum) * 6, (GPS_DDNum + BDS_DDNum) * 6, HPHT, R, HPHTR);
	MatrixInverse(((GPS_DDNum + BDS_DDNum) * 6), (GPS_DDNum + BDS_DDNum) * 6, HPHTR, INHPHTR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), ((GPS_DDNum + BDS_DDNum) * 6), (GPS_DDNum + BDS_DDNum) * 6, PHT, INHPHTR, K);
	MatrixMultiply((GPS_DDNum + BDS_DDNum) * 6, (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), 1, H, X, FX);
	MatrixSub((GPS_DDNum + BDS_DDNum) *6, 1, Z, FX, ZFX);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (GPS_DDNum + BDS_DDNum) * 6, (GPS_DDNum + BDS_DDNum) * 6, 1, K, ZFX, KZFX);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 1, X, KZFX, X);
	MatrixTranspose((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), K, KT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), ((GPS_DDNum + BDS_DDNum) * 6), (3 + (GPS_DDNum + BDS_DDNum) * 2), K, H, KH);
	MatrixSub(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, I, KH, IKH);
	MatrixTranspose(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, IKHT);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, P, IKHP);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKHP, IKHT, P);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), ((GPS_DDNum + BDS_DDNum) * 6), K, R, KR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2 * 2 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 6), (3 + (GPS_DDNum + BDS_DDNum) * 2), KR, KT, KRKT);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, P, KRKT, P);
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		EkfRTK->X[i] = X[i];
		if (i < 3)RTKPos_Rover->Pos[i] = X[i];
		if (i >= 3)EkfRTK->Ambi[i - 3] = X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j] = P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j];
	}
	RTKPos_Rover->PDOP = sqrt(P[0] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 1 + 1] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 2 + 2]);
	delete[]X; delete[] P; delete[] I; delete[] Z; delete[]H; delete[]FX; delete[] R;
	delete[]HT; delete[]PHT; delete[]HPHT; delete[] HPHTR; delete[]INHPHTR; delete[] K;  delete[]ZFX;  delete[]KZFX; delete[]KT; delete[] KH; delete[]IKH; delete[]IKHT; delete[]IKHP; delete[]KR; delete[]KRKT;
	return ;
}



void KF_FIXED2(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK, double Fixed[])
{
	int GPS_DDNum = 0, BDS_DDNum = 0;
	int GPS_Tra = 0, BDS_Tra = 0;
	double* X, * P, * I, * Z, * H, * FX, * R;
	double* HT, * PHT, * HPHT, * HPHTR, * INHPHTR, * K, * ZFX, * KZFX, * KT, * KH, * IKH, * IKHT, * IKHP, * KR, * KRKT;
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		if (SDObs->SdSatObs[i].Valid == false) continue;
		if (SDObs->SdSatObs[i].System == GPS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0])GPS_DDNum++;
		if (SDObs->SdSatObs[i].System == BDS && SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1])BDS_DDNum++;
	}
	X = new double[3 + (GPS_DDNum + BDS_DDNum) * 2] {0.0};
	P = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	Z = new double[(GPS_DDNum + BDS_DDNum) * 2] {0.0};
	I = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	H = new double[((GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	FX = new double[(GPS_DDNum + BDS_DDNum) * 2] {0.0};
	R = new double[((GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	HT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	PHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	HPHT = new double[((GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	HPHTR = new double[((GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	INHPHTR = new double[((GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	K = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	ZFX = new double[(GPS_DDNum + BDS_DDNum) * 2] {0.0};
	KZFX = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KT = new double[((GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKH = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKHT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	IKHP = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KR = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	KRKT = new double[(3 + (GPS_DDNum + BDS_DDNum) * 2) * (3 + (GPS_DDNum + BDS_DDNum) * 2)] {0.0};
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		int index = 0;
		if (rawdata->RoverObs.SatPVT[SDObs->SdSatObs[i].iRov].Valid == false || rawdata->BaseObs.SatPVT[SDObs->SdSatObs[i].iBas].Valid == false) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[0] && SDObs->SdSatObs[i].System == GPS) continue;
		if (SDObs->SdSatObs[i].Prn == DDObs->RefPrn[1] && SDObs->SdSatObs[i].System == BDS) continue;
		if (SDObs->SdSatObs[i].System == GPS) { index = 0; GPS_Tra++; }
		else if (SDObs->SdSatObs[i].System == BDS) { index = 1; BDS_Tra++; }
		Z[((index == 0) ? (GPS_Tra - 1) * 2 :(GPS_DDNum + BDS_Tra - 1) * 2)] = Fixed[(index == 0) ? (GPS_Tra - 1) * 2 : (GPS_DDNum + BDS_Tra - 1) * 2];
		Z[((index == 0) ? (GPS_Tra - 1) * 2 + 1 : (GPS_DDNum + BDS_Tra - 1) * 2 + 1)] = Fixed[(index == 0) ? (GPS_Tra - 1) * 2 + 1 : (GPS_DDNum + BDS_Tra - 1) * 2 + 1];
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * (GPS_DDNum + BDS_Tra - 1)) + 3 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = 1.0;
		H[(3 + (GPS_DDNum + BDS_DDNum) * 2) * ((index == 0) ? 2 * (GPS_Tra - 1) + 1 : 2 * (GPS_DDNum + BDS_Tra - 1) + 1) + 4 + ((index == 0) ? 2 * (GPS_Tra - 1) : 2 * GPS_DDNum + 2 * (BDS_Tra - 1))] = 1.0;
	}
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		I[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + i] = 1.0;
		X[i] = EkfRTK->X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j] = EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j];
	}
	for (int i = 0; i < (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		for (int j = 0; j < (GPS_DDNum + BDS_DDNum) * 2; j++)
		{
			if (i == j)
			{
				R[((GPS_DDNum + BDS_DDNum) * 2) * i + j] = 1e-10;
			}
		}
	}
	MatrixTranspose((GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, H, HT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), P, HT, PHT);
	MatrixMultiply((GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, (3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), H, PHT, HPHT);
	MatrixAdd((GPS_DDNum + BDS_DDNum) * 2, (GPS_DDNum + BDS_DDNum) * 2, HPHT, R, HPHTR);
	MatrixInverse(((GPS_DDNum + BDS_DDNum) * 2), (GPS_DDNum + BDS_DDNum) * 2, HPHTR, INHPHTR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), (GPS_DDNum + BDS_DDNum) * 2, PHT, INHPHTR, K);
	MatrixMultiply((GPS_DDNum + BDS_DDNum) * 2, (3 + (GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), 1, H, X, FX);
	MatrixSub((GPS_DDNum + BDS_DDNum) * 2, 1, Z, FX, ZFX);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), (GPS_DDNum + BDS_DDNum) * 2, (GPS_DDNum + BDS_DDNum) * 2, 1, K, ZFX, KZFX);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 1, X, KZFX, X);
	MatrixTranspose((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), K, KT);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), K, H, KH);
	MatrixSub(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, I, KH, IKH);
	MatrixTranspose(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, IKHT);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKH, P, IKHP);
	MatrixMultiply(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, IKHP, IKHT, P);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), K, R, KR);
	MatrixMultiply((3 + (GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), ((GPS_DDNum + BDS_DDNum) * 2), (3 + (GPS_DDNum + BDS_DDNum) * 2), KR, KT, KRKT);
	MatrixAdd(3 + (GPS_DDNum + BDS_DDNum) * 2, 3 + (GPS_DDNum + BDS_DDNum) * 2, P, KRKT, P);
	for (int i = 0; i < 3 + (GPS_DDNum + BDS_DDNum) * 2; i++)
	{
		EkfRTK->X[i] = X[i];
		if (i < 3)RTKPos_Rover->Pos[i] = X[i];
		if (i >= 3)EkfRTK->Ambi[i - 3] = X[i];
		for (int j = 0; j < 3 + (GPS_DDNum + BDS_DDNum) * 2; j++)EkfRTK->P[i * (3 + (MAXCHANNUM - 2) * 2) + j] = P[i * (3 + (GPS_DDNum + BDS_DDNum) * 2) + j];
	}
	RTKPos_Rover->PDOP = sqrt(P[0] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 1 + 1] + P[(3 + (GPS_DDNum + BDS_DDNum) * 2) * 2 + 2]);
	delete[]X; delete[] P; delete[] I; delete[] Z; delete[]H; delete[]FX; delete[] R;
	delete[]HT; delete[]PHT; delete[]HPHT; 
	delete[] HPHTR; delete[]INHPHTR; delete[] K;  
	delete[]ZFX;  delete[]KZFX; delete[]KT; 
	delete[] KH; delete[]IKH; delete[]IKHT; 
	delete[]IKHP; delete[]KR; delete[]KRKT;
	return;
}