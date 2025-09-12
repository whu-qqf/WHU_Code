#include"SPP.h"
#include<cmath>
#include"iostream"
#include"Matrix.h"
#include"SatPos.h"
#include"const.h"

/**
* @brief 计算卫星信号发射时刻位置
* @param Epk 观测数据结构体指针
* @param Eph GPS历元数据结构体指针
* @param BDSEph BDS历元数据结构体指针
* @param UserPos 用户位置
*/
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3])
{
    /*计算卫星信号发射时刻位置*/
    double      PIF;
    int         prn;
	GPSTIME     t_trans, t_rec;    //信号发射时刻和信号接收时刻
	GPSEPHREC*  GPSeph, * BDSeph;
    memset(Epk->SatPVT, 0, sizeof(Epk->SatPVT));
    for (int i = 0; i < Epk->SatNum; i++)
    {
        if (Epk->SatObs[i].Valid == false)
        {
			Epk->SatPVT[i].Valid = false;
            continue;
        }
        if (Epk->SatObs[i].System == GPS)
        {
            PIF =    Epk->ComObs[i].PIF;
			prn =    Epk->SatObs[i].Prn;
            GPSeph = Eph + prn - 1;
            t_rec =  Epk->Time;
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF /C_Light;
            t_trans.Week = t_rec.Week;
            if (!CompSatClkOff(prn, GPS, &t_trans, Eph, BDSEph, &Epk->SatPVT[i])) 
            {
                Epk->SatPVT[i].Valid = false;
                continue;
            }
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epk->SatPVT[i].SatClkOft;
            CompSatClkOff(prn, GPS, &t_trans, Eph, BDSEph, &Epk->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epk->SatPVT[i].SatClkOft;
            CompGPSSatPVT(prn, &t_trans, GPSeph, &(Epk->SatPVT[i]));
            EarthRotate(UserPos, GPSeph, &Epk->SatPVT[i]);
			Epk->SatPVT[i].Valid = true;
        }
        if (Epk->SatObs[i].System == BDS)
        {
            PIF = Epk->ComObs[i].PIF;
            prn = Epk->SatObs[i].Prn;
			BDSeph = BDSEph + prn - 1;
            t_rec = Epk->Time;
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light;
            if (!CompSatClkOff(prn, BDS, &t_trans, Eph, BDSEph, &Epk->SatPVT[i])) 
            {
                Epk->SatPVT[i].Valid = false;
                continue;
            }
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epk->SatPVT[i].SatClkOft;
            CompSatClkOff(prn, BDS, &t_trans, Eph, BDSEph, &Epk->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epk->SatPVT[i].SatClkOft;
            CompBDSSatPVT(prn, &t_trans, BDSeph, &(Epk->SatPVT[i]));
            EarthRotate(UserPos, BDSeph, &Epk->SatPVT[i]);
            if ((BDSeph)->PRN == 0)Epk->SatPVT[i].Valid = false;
            else Epk->SatPVT[i].Valid = true;
            if ((BDSeph)->SVHealth == 1)Epk->SatPVT[i].Valid = false;
        }
    }
}

/**
* @brief 单点定位计算函数
* @param Epoch 观测数据结构体指针
* @param GPSEph GPS历元数据结构体指针
* @param BDSEph BDS历元数据结构体指针
* @param Res 位置解算结果结构体指针
* @return true 定位成功, false 失败
*/
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)
{
	/*单点定位计算函数*/
    double B[5 * MAXCHANNUM] = { 0 }, w[MAXCHANNUM] = { 0 };
    double B_[4 * MAXCHANNUM] = { 0 };
    double SatPos[3],pos[3];
    double UsePos[5] = { 0 }, UserPos[5] = { 0 };
    double MidPos[5] = { 0 };
    double PDOP = 0.0, sigma = 0.0;
	for (int i = 0; i < 3; i++) UsePos[i] = Res->Pos[i] ,pos[i]=Res->Pos[i];
	UsePos[3] = Res->GPS_clockBias ;
	UsePos[4] = Res->BDS_clockBias ;
    double distance=0,tgd = 0;
	bool is_BDS = false, is_GPS = false;
    int j = 0;
    do {
        j = 0;
        ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, pos);
        for (int i = 0; i < Epoch->SatNum; i++)
        {
            if (Epoch->SatPVT[i].Valid == false)continue;
			for (int q = 0; q < 5; q++)UserPos[q] = UsePos[q] ;
            for (int k = 0; k < 3; k++) SatPos[k] = Epoch->SatPVT[i].SatPos[k];
            double Rou = sqrt((SatPos[0] - UsePos[0]) * (SatPos[0] - UsePos[0]) + (SatPos[1] - UsePos[1]) * (SatPos[1] - UsePos[1]) + (SatPos[2] - UsePos[2]) * (SatPos[2] - UsePos[2]));
            double l = -(SatPos[0] - UsePos[0]) / Rou, m = -(SatPos[1] - UsePos[1]) / Rou, n = -(SatPos[2] - UsePos[2]) / Rou;
            *(B + 5 * j + 0) = l, * (B + 5 * j + 1) = m, * (B + 5 * j + 2) = n;
            if (Epoch->SatObs[i].System == GPS)
            {
                tgd = 0;
                *(B + 5 * j + 3) = 1;
                *(B + 5 * j + 4) = 0;
                //w[j] = Epoch->ComObs[i].PIF - Rou + Epoch->SatPVT[i].SatClkOft * C_Light - Epoch->SatPVT[i].TropCorr - tgd - UsePos[3] ;

                is_GPS = true;
            }
            else if (Epoch->SatObs[i].System == BDS)
            {
                GPSEPHREC* BDSeph = BDSEph + Epoch->SatObs[i].Prn - 1;
                tgd = C_Light*BDSeph->TGD1* FG1_BDS* FG1_BDS/(FG1_BDS* FG1_BDS- FG3_BDS* FG3_BDS);
                *(B + 5 * j + 3) = 0;
                *(B + 5 * j + 4) = 1;
                //w[j] = Epoch->ComObs[i].PIF - Rou + Epoch->SatPVT[i].SatClkOft * C_Light - Epoch->SatPVT[i].TropCorr - tgd - UsePos[4];

                is_BDS = true;
            }
            /*if (!is_BDS || !is_GPS)
            {
                w[j] = Epoch->ComObs[i].PIF - Rou + Epoch->SatPVT[i].SatClkOft * C_Light - Epoch->SatPVT[i].TropCorr - tgd - UsePos[3];
            }*/
            w[j] = Epoch->ComObs[i].PIF - Rou + l * UsePos[0] + m * UsePos[1] + n * UsePos[2] + Epoch->SatPVT[i].SatClkOft * C_Light - Epoch->SatPVT[i].TropCorr - tgd;

            j++;
        }
        if (j < 4) return false;
        if (!is_BDS || !is_GPS)
        {
            if (!is_BDS)
            {
                int cols[1] = { 4 };
                int rows[1] = {};
                MatrixExtr(MAXCHANNUM, 5, rows, 0, cols, 1, B, B_);
            }
            if (!is_GPS)
            {
                int cols[1] = { 3 };
                int rows[1] = {};
                MatrixExtr(MAXCHANNUM, 5, rows, 0, cols, 1, B, B_);
            }
            double BT[4 * MAXCHANNUM] = { 0 };
            double BTB[4 * 4] = { 0 };
            double BTBIN[4 * 4] = { 0 };
            double BM[4 * MAXCHANNUM] = { 0 };
            MatrixTranspose(MAXCHANNUM, 4, B_, BT);
            MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 4, BT, B_, BTB);
            if(!MatrixInverse(4, BTB, BTBIN))return false;
            MatrixMultiply(4, 4, 4, MAXCHANNUM, BTBIN, BT, BM);
            MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 1, BM, w, UsePos);
			//MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 1, BM, w, MidPos);
            ///////////////
			//MatrixAdd(5, 1, MidPos, UsePos, UsePos);
            if (!is_BDS)
            {
                Res->BDS_clockBias = 0;
                Res->GPS_clockBias = UsePos[3];
            }
            else if (!is_GPS)
            {
                Res->GPS_clockBias = 0;
                Res->BDS_clockBias = UsePos[3];
            }
            PDOP = sqrt(BTBIN[0]+BTBIN[5]+BTBIN[10]);
        }
        else
        {
            double BT[5 * MAXCHANNUM] = { 0 };
            double BTB[5 * 5] = { 0 };
            double BTBIN[5 * 5] = { 0 };
            double BM[5 * MAXCHANNUM] = { 0 };
            MatrixTranspose(MAXCHANNUM, 5, B, BT);
            MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 5, BT, B, BTB);
            if(!MatrixInverse(5, BTB, BTBIN))return false;
            MatrixMultiply(5, 5, 5, MAXCHANNUM, BTBIN, BT, BM);
            MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 1, BM, w, UsePos);
			//MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 1, BM, w, MidPos);
			///////////////
			//MatrixAdd(5, 1, MidPos, UsePos, UsePos);
            Res->GPS_clockBias = UsePos[3], Res->BDS_clockBias = UsePos[4];
            PDOP = sqrt(BTBIN[0] + BTBIN[6] + BTBIN[12]);
        }
        double BX[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, VT[MAXCHANNUM] = { 0 }, VTV[1] = { 0 };
		MatrixMultiply(MAXCHANNUM, 5, 5, 1, B, UsePos, BX);
        MatrixSub(MAXCHANNUM, 1, BX, w, V);
		MatrixTranspose(MAXCHANNUM, 1, V, VT);
		MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, VT, V, VTV);
		sigma = sqrt(VTV[0] / (j - 5));

        distance = sqrt((UserPos[0] - UsePos[0]) * (UserPos[0] - UsePos[0]) + (UserPos[1] - UsePos[1]) * (UserPos[1] - UsePos[1]) + (UserPos[2] - UsePos[2]) * (UserPos[2] - UsePos[2]));
        for (int r = 0; r < 3; r++)pos[r] = UsePos[r];
    } while (distance>1e-4);
	Res->PDOP = PDOP;
    Res->SigmaPos = sigma;
	Res->SatNum = Epoch->SatNum;
    for (int i = 0; i < 3; i++)
    {
        Res->Pos[i] = UsePos[i];
    }
    return true;
}


/**
* @brief 卫星速度计算函数
* @param Epoch 观测数据结构体指针
* @param Res 位置解算结果结构体指针
*/
void SPV(EPOCHOBSDATA* Epoch, POSRES* Res)
{
	/*卫星速度计算函数*/ 
    double B[MAXCHANNUM * 4] = { 0 }, w[MAXCHANNUM] = { 0 }, BX[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, VT[MAXCHANNUM] = { 0 }, VTV[1] = { 0 };
    double BT[4 * MAXCHANNUM] = { 0 }, BTB[4 * 4] = { 0 }, BTBIN[4 * 4] = { 0 }, BM[4 * MAXCHANNUM] = { 0 };
    double SatPos[3] = { 0 }, RecPos[3] = { 0 }, SatVel[3] = { 0 }, RecVel[4] = { 0 };
    int j = 0;
    for (int i = 0; i < Epoch->SatNum; i++)
    {
        if (Epoch->SatPVT[i].Valid == false)continue;
        for (int k = 0; k < 3; k++)SatPos[k] = Epoch->SatPVT[i].SatPos[k], RecPos[k] = Res->Pos[k], SatVel[k] = Epoch->SatPVT[i].SatVel[k];
        double Rou = sqrt((SatPos[0] - RecPos[0]) * (SatPos[0] - RecPos[0]) + (SatPos[1] - RecPos[1]) * (SatPos[1] - RecPos[1]) + (SatPos[2] - RecPos[2]) * (SatPos[2] - RecPos[2]));
        double l = -(SatPos[0] - RecPos[0]) / Rou, m = -(SatPos[1] - RecPos[1]) / Rou, n = -(SatPos[2] - RecPos[2]) / Rou;
        B[4 * j + 0] = l, B[4 * j + 1] = m, B[4 * j + 2] = n, B[4 * j + 3] = 1;
        w[j] = Epoch->SatObs[i].D[0] - ((SatPos[0] - RecPos[0]) * SatVel[0] + (SatPos[1] - RecPos[1]) * SatVel[1] + (SatPos[2] - RecPos[2]) * SatVel[2]) / Rou + C_Light * Epoch->SatPVT[i].SatClkSft;
        j++;
    }
    if (j < 4)return;
    MatrixTranspose(MAXCHANNUM, 4, B, BT);
    MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 4, BT, B, BTB);
    if(!MatrixInverse(4, BTB, BTBIN))return;
    MatrixMultiply(4, 4, 4, MAXCHANNUM, BTBIN, BT, BM);
    MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 1, BM, w, RecVel);
	MatrixMultiply(MAXCHANNUM, 4, 4, 1, B, RecVel, BX);
	MatrixSub(MAXCHANNUM, 1, BX, w, V);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, V, V, VTV);
	Res->SigmaVel = sqrt(VTV[0] / (j - 4));
    for (int z = 0; z < 3; z++)Res->Vel[z] = RecVel[z];
}