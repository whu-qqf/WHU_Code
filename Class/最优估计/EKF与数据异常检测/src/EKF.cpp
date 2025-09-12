#include<cmath>
#include<iostream>
#include"struct.h"
#include"Matrix.h"
#include "EKF.h"
#include"CoordinateConvert.h"
#include"DetectError.h"

using namespace std;

void EKF_5(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu)
{
	EKFOBS* obs;
	int j = 0;
	double H[MAXCHANNUM * 5] = { 0 }, W[MAXCHANNUM * 5] = { 0 }, D_delta[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double Z[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, F[MAXCHANNUM] = { 0 };
	double K[MAXCHANNUM * 5] = { 0 };
	double KH[5 * 5] = { 0 };
	double KV[MAXCHANNUM] = { 0 }, I_KH[5 * 5] = { 0 };
	double DHT[5 * MAXCHANNUM] = { 0 }, HT[5 * MAXCHANNUM] = { 0 };
	double R[MAXCHANNUM * MAXCHANNUM] = { 0 }, RIN[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double HDHT[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double I[5 * 5] = { 1.0,0,0,0,0,
	0,1.0,0,0,0,
	0,0,1.0,0,0,
	0,0,0,1.0,0,
	0,0,0,0,1.0};
	double DW[5 * 5] = { 300,0,0,0,0,
	0,300,0,0,0,
	0,0,300,0,0,
	0,0,0,300,0,
	0,0,0,0,300};
	MatrixAdd(5, 5, D, DW, D); 
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		obs = Obs + i;
		if (obs->Prn == 0) continue; 
		double Rou=sqrt((X[0]- obs->X) * (X[0] - obs->X) +(X[1] - obs->Y) * (X[1] - obs->Y) +(X[2] - obs->Z) * (X[2] - obs->Z));
		H[5 * j + 0] = (X[0] - obs->X) / Rou, H[5 * j + 1] = (X[1] - obs->Y) / Rou, H[5 * j + 2] = (X[2] - obs->Z) / Rou;
		if(obs->Sys==GPS)H[5*j + 3] = 1, H[5 * j + 4] = 0; 
		else if(obs->Sys==BDS)H[5 * j + 3] = 0, H[5 * j + 4] = 1; 
		D_delta[MAXCHANNUM * j + j] = obs->error;
		Z[j] = obs->P;
		F[j] = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z)) + H[5 * j + 3] * X[3] + H[5 * j + 4] * X[4];
		V[j] = Z[j] - F[j];
		j++;
	}
	MatrixTranspose(MAXCHANNUM, 5, H, HT);
	MatrixMultiply(5, 5, 5, MAXCHANNUM, D, HT, DHT);
	MatrixMultiply(MAXCHANNUM, 5, 5, MAXCHANNUM, H, DHT, HDHT);
	MatrixAdd(MAXCHANNUM, MAXCHANNUM, HDHT, D_delta, R);
	MatrixInverse(MAXCHANNUM, j, R, RIN);
	MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, DHT, RIN, K);
	MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 1, K, V, KV);
	MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 5, K, H, KH);
	MatrixSub(5, 5, I, KH, I_KH); //I_KH = I - KH
	MatrixAdd(5, 1, X, KV, X); //X = X + KV
	MatrixMultiply(5, 5, 5, 5, I_KH, D, D); //D = (I - KH) * D
	XYZ xyz;
	GEOCOOR ref_geo;
	xyz.x = X[0], xyz.y = X[1], xyz.z = X[2];
	ref_geo.latitude = refPos->blh[0]*PI/180, ref_geo.longitude = refPos->blh[1]*PI/180, ref_geo.height = refPos->blh[2];
	ToENU(ref_geo, xyz, enu, R_WGS84, E_WGS84);
}

void EKF_8(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu)
{
	EKFOBS* obs;
	int j = 0;
	double q_2 = 300.0, q_BDS = 1, q_GPS = 1; //误差方差
	double PHIT[8 * 8] = { 0 }, PHID[8 * 8] = { 0 },PDPT[8 * 8] = { 0 };
	double PHI[8*8]={1,0,0,1,0,0,0,0,   //转移矩阵
				 0,1,0,0,1,0,0,0,
				 0,0,1,0,0,1,0,0,
				 0,0,0,1,0,0,0,0,
				 0,0,0,0,1,0,0,0,
				 0,0,0,0,0,1,0,0,
	             0,0,0,0,0,0,1,0,
	             0,0,0,0,0,0,0,1};
	double I[8 * 8] = { 1,0,0,0,0,0,0,0,
		0,1,0,0,0,0,0,0,
		0,0,1,0,0,0,0,0,
		0,0,0,1,0,0,0,0,
		0,0,0,0,1,0,0,0,
		0,0,0,0,0,1,0,0,
		0,0,0,0,0,0,1,0,
		0,0,0,0,0,0,0,1 };
	double Dw[8 * 8] = { q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,
		0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,
		0,0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,
		q_2 / 2.0,0,0,q_2,0,0,0,0,
		0,q_2 / 2.0,0,0,q_2,0,0,0,
		0,0,q_2 / 2.0,0,0,q_2,0,0,
		0,0,0,0,0,0,q_GPS,0,
		0,0,0,0,0,0,0,q_BDS
	};
	//时间预测
	MatrixMultiply(8, 8, 8, 1, PHI, X, X);
	MatrixTranspose(8, 8, PHI, PHIT);
	MatrixMultiply(8, 8, 8, 8, PHI, D, PHID);
	MatrixMultiply(8, 8, 8, 8, PHID, PHIT, PDPT);
	MatrixAdd(8, 8, PDPT, Dw, D);
	double H[MAXCHANNUM * 8] = { 0 }, W[MAXCHANNUM * 8] = { 0 }, D_delta[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double Z[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, F[MAXCHANNUM] = { 0 };
	double HT[8 * MAXCHANNUM] = { 0 },DHT[8 * MAXCHANNUM] = { 0 };
	double HDHT[MAXCHANNUM * MAXCHANNUM] = { 0 }, R[MAXCHANNUM * MAXCHANNUM] = { 0 }, RIN[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double K[MAXCHANNUM * 8] = { 0 }, KV[8] = { 0 }, KH[8 * 8] = { 0 }, I_KH[8 * 8] = { 0 };
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		obs = Obs + i;
		if (obs->Prn == 0) continue;
		double Rou = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z));
		H[8 * j + 0] = (X[0] - obs->X) / Rou, H[8 * j + 1] = (X[1] - obs->Y) / Rou, H[8 * j + 2] = (X[2] - obs->Z) / Rou;
		H[8 * j + 3] = H[8 * j + 4] = H[8 * j + 5] = 0;
		if (obs->Sys == GPS)H[8 * j + 6] = 1, H[8 * j + 7] = 0;
		else if (obs->Sys == BDS)H[8 * j + 7] = 0, H[8 * j + 7] = 1;
		D_delta[MAXCHANNUM * j + j] = obs->error;
		Z[j] = obs->P;
		F[j] = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z)) + H[8 * j + 6] * X[6] + H[8 * j + 7] * X[7];
		V[j] = Z[j] - F[j];
		j++;
	}
	MatrixTranspose(MAXCHANNUM, 8, H, HT);
	MatrixMultiply(8, 8, 8, MAXCHANNUM, D, HT, DHT);
	MatrixMultiply(MAXCHANNUM, 8, 8, MAXCHANNUM, H, DHT, HDHT);
	MatrixAdd(MAXCHANNUM, MAXCHANNUM, HDHT, D_delta, R);
	MatrixInverse(MAXCHANNUM, j, R, RIN);
	MatrixMultiply(8, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, DHT, RIN, K);
	MatrixMultiply(8, MAXCHANNUM, MAXCHANNUM, 1, K, V, KV);
	MatrixMultiply(8, MAXCHANNUM, MAXCHANNUM, 8, K, H, KH);
	MatrixSub(8, 8, I, KH, I_KH); //I_KH = I - KH
	MatrixAdd(8, 1, X, KV, X); //X = X + KV
	MatrixMultiply(8, 8, 8, 8, I_KH, D, D); //D = (I - KH) * D
	XYZ xyz;
	GEOCOOR ref_geo;
	xyz.x = X[0], xyz.y = X[1], xyz.z = X[2];
	ref_geo.latitude = refPos->blh[0] * PI / 180, ref_geo.longitude = refPos->blh[1] * PI / 180, ref_geo.height = refPos->blh[2];
	ToENU(ref_geo, xyz, enu, R_WGS84, E_WGS84);
	
	
}

void EKF_9(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu)
{
	EKFOBS* obs;
	int j = 0;
	double q_2 = 2.25, q_clk=1e-5; //误差方差
	double PHIT[9 * 9] = { 0 }, PHID[9 * 9] = { 0 }, PDPT[9 * 9] = { 0 };
	double PHI[9 * 9] = { 1,0,0,1,0,0,0,0,0,   //转移矩阵
				 0,1,0,0,1,0,0,0,0,
				 0,0,1,0,0,1,0,0,0,
				 0,0,0,1,0,0,0,0,0,
				 0,0,0,0,1,0,0,0,0,
				 0,0,0,0,0,1,0,0,0,
				 0,0,0,0,0,0,1,0,1,
				 0,0,0,0,0,0,0,1,1,
	             0,0,0,0,0,0,0,0,1};
	double I[9 * 9] = { 1,0,0,0,0,0,0,0,0,
			0,1,0,0,0,0,0,0,0,
			0,0,1,0,0,0,0,0,0,
			0,0,0,1,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,
			0,0,0,0,0,1,0,0,0,
			0,0,0,0,0,0,1,0,0,
			0,0,0,0,0,0,0,1,0,
			0,0,0,0,0,0,0,0,1
	};
	double Dw[9 * 9] = { q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,0,
		0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,
		0,0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,
		q_2 / 2.0,0,0,q_2,0,0,0,0,0,
		0,q_2 / 2.0,0,0,q_2,0,0,0,0,
		0,0,q_2 / 2.0,0,0,q_2,0,0,0,
		0,0,0,0,0,0,q_clk / 3.0,q_clk / 3.0,q_clk / 2.0,
		0,0,0,0,0,0,q_clk / 3.0,q_clk / 3.0,q_clk / 2.0,
		0,0,0,0,0,0,q_clk / 2.0,q_clk / 2.0,q_clk
	};
	//时间预测
	MatrixMultiply(9, 9, 9, 1, PHI, X, X);
	MatrixTranspose(9, 9, PHI, PHIT);
	MatrixMultiply(9, 9, 9, 9, PHI, D, PHID);
	MatrixMultiply(9, 9, 9, 9, PHID, PHIT, PDPT);
	MatrixAdd(9, 9, PDPT, Dw, D);
	double H[MAXCHANNUM * 9] = { 0 }, W[MAXCHANNUM * 9] = { 0 }, D_delta[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double Z[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, F[MAXCHANNUM] = { 0 };
	double HT[9 * MAXCHANNUM] = { 0 }, DHT[9 * MAXCHANNUM] = { 0 };
	double HDHT[MAXCHANNUM * MAXCHANNUM] = { 0 }, R[MAXCHANNUM * MAXCHANNUM] = { 0 }, RIN[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double K[MAXCHANNUM * 9] = { 0 }, KV[9] = { 0 }, KH[9 * 9] = { 0 }, I_KH[9 * 9] = { 0 };
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		obs = Obs + i;
		if (obs->Prn == 0) continue;
		double Rou = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z));
		H[9 * j + 0] = (X[0] - obs->X) / Rou, H[9 * j + 1] = (X[1] - obs->Y) / Rou, H[9 * j + 2] = (X[2] - obs->Z) / Rou;
		H[9 * j + 3] = H[9 * j + 4] = H[9 * j + 5] = 0;
		H[9 * j + 8] = 0;
		if (obs->Sys == GPS)H[9 * j + 6] = 1, H[9 * j + 7] = 0;
		else if (obs->Sys == BDS)H[9 * j + 6] = 0, H[9 * j + 7] = 1;
		D_delta[MAXCHANNUM * j + j] = obs->error;
		Z[j] = obs->P;
		F[j] = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z)) + H[9 * j + 6] * X[6] + H[9 * j + 7] * X[7];
		V[j] = Z[j] - F[j];
		j++;
	}
	MatrixTranspose(MAXCHANNUM, 9, H, HT);
	MatrixMultiply(9, 9, 9, MAXCHANNUM, D, HT, DHT);
	MatrixMultiply(MAXCHANNUM, 9, 9, MAXCHANNUM, H, DHT, HDHT);
	MatrixAdd(MAXCHANNUM, MAXCHANNUM, HDHT, D_delta, R);
	MatrixInverse(MAXCHANNUM, j, R, RIN);
	MatrixMultiply(9, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, DHT, RIN, K);
	MatrixMultiply(9, MAXCHANNUM, MAXCHANNUM, 1, K, V, KV);
	MatrixMultiply(9, MAXCHANNUM, MAXCHANNUM, 9, K, H, KH);
	MatrixSub(9, 9, I, KH, I_KH); //I_KH = I - KH
	MatrixAdd(9, 1, X, KV, X); //X = X + KV
	MatrixMultiply(9, 9, 9, 9, I_KH, D, D); //D = (I - KH) * D
	XYZ xyz;
	GEOCOOR ref_geo;
	xyz.x = X[0], xyz.y = X[1], xyz.z = X[2];
	ref_geo.latitude = refPos->blh[0] * PI / 180, ref_geo.longitude = refPos->blh[1] * PI / 180, ref_geo.height = refPos->blh[2];
	ToENU(ref_geo, xyz, enu, R_WGS84, E_WGS84);
}

void EKF_10(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu)
{
	EKFOBS* obs;
	int j = 0;
	double q_2 = 300, q_dGPS = 1, q_dBDS = 1; //误差方差
	double PHIT[10 * 10] = { 0 }, PHID[10 * 10] = { 0 }, PDPT[10 * 10] = { 0 };
	double PHI[10 * 10] = { 1,0,0,1,0,0,0,0,0,0,   //转移矩阵
				 0,1,0,0,1,0,0,0,0,0,
				 0,0,1,0,0,1,0,0,0,0,
				 0,0,0,1,0,0,0,0,0,0,
				 0,0,0,0,1,0,0,0,0,0,
				 0,0,0,0,0,1,0,0,0,0,
				 0,0,0,0,0,0,1,0,1,0,
				 0,0,0,0,0,0,0,1,0,1,
				 0,0,0,0,0,0,0,0,1,0,
				 0,0,0,0,0,0,0,0,0,1
	};
	double I[10 * 10] = { 1,0,0,0,0,0,0,0,0,0,
			0,1,0,0,0,0,0,0,0,0,
			0,0,1,0,0,0,0,0,0,0,
			0,0,0,1,0,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,0,
			0,0,0,0,0,1,0,0,0,0,
			0,0,0,0,0,0,1,0,0,0,
			0,0,0,0,0,0,0,1,0,0,
			0,0,0,0,0,0,0,0,1,0,
			0,0,0,0,0,0,0,0,0,1
	};
	double Dw[10 * 10] = { q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,0,0,
		0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,0,
		0,0,q_2 / 3.0,0,0,q_2 / 2.0,0,0,0,0,
		q_2 / 2.0,0,0,q_2,0,0,0,0,0,0,
		0,q_2 / 2.0,0,0,q_2,0,0,0,0,0,
		0,0,q_2 / 2.0,0,0,q_2,0,0,0,0,
		0,0,0,0,0,0,q_dGPS / 3,0,q_dGPS / 2.0,0,
		0,0,0,0,0,0,0,q_dBDS / 3,0,q_dBDS / 2.0,
		0,0,0,0,0,0,q_dGPS / 2.0,0,q_dGPS,0,
		0,0,0,0,0,0,0,q_dBDS / 2.0,0,q_dBDS
	};
	//时间预测
	MatrixMultiply(10, 10, 10, 1, PHI, X, X);
	MatrixTranspose(10, 10, PHI, PHIT);
	MatrixMultiply(10, 10, 10, 10, PHI, D, PHID);
	MatrixMultiply(10, 10, 10, 10, PHID, PHIT, PDPT);
	MatrixAdd(10, 10, PDPT, Dw, D);
	double H[MAXCHANNUM * 10] = { 0 }, D_delta[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double Z[MAXCHANNUM] = { 0 }, V[MAXCHANNUM] = { 0 }, F[MAXCHANNUM] = { 0 };
	double HT[10 * MAXCHANNUM] = { 0 }, DHT[10 * MAXCHANNUM] = { 0 };
	double HDHT[MAXCHANNUM * MAXCHANNUM] = { 0 }, R[MAXCHANNUM * MAXCHANNUM] = { 0 }, RIN[MAXCHANNUM * MAXCHANNUM] = { 0 };
	double K[MAXCHANNUM * 10] = { 0 }, KV[10] = { 0 }, KH[10 * 10] = { 0 }, I_KH[10 * 10] = { 0 };
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		obs = Obs + i;
		if (obs->Prn == 0) continue;
		double Rou = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z));
		H[10 * j + 0] = (X[0] - obs->X) / Rou, H[10 * j + 1] = (X[1] - obs->Y) / Rou, H[10 * j + 2] = (X[2] - obs->Z) / Rou;
		H[10 * j + 3] = H[10 * j + 4] = H[10 * j + 5] = H[10 * j + 8] = H[10 * j + 9] = 0;
		if (obs->Sys == GPS)H[10 * j + 6] = 1, H[10 * j + 7] = 0;
		else if (obs->Sys == BDS)H[10 * j + 6] = 0, H[10 * j + 7] = 1;
		D_delta[MAXCHANNUM * j + j] = obs->error;
		Z[j] = obs->P;
		F[j] = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z)) + H[10 * j + 6] * X[6] + H[10 * j + 7] * X[7];
		V[j] = Z[j] - F[j];
		j++;
	}
	//剔除异常观测值
	MatrixTranspose(MAXCHANNUM, 10, H, HT);
	MatrixMultiply(10, 10, 10, MAXCHANNUM, D, HT, DHT);
	MatrixMultiply(MAXCHANNUM, 10, 10, MAXCHANNUM, H, DHT, HDHT);
	MatrixAdd(MAXCHANNUM, MAXCHANNUM, HDHT, D_delta, R);
	MatrixInverse(MAXCHANNUM, j, R, RIN);
	int needdelete[1] = { -1 };
	double t = 0.0;
	for (int i = 0; i < j; i++)
	{
		if (SingleError(V, RIN, i) > 1.645)
		{
			if (SingleError(V, RIN, i) > t)
			{
				needdelete[0] = i;
				t = SingleError(V, RIN, i);
			}
		}
	}
	j = 0;
	memset(V, 0, sizeof(V));
	memset(Z, 0, sizeof(Z));
	memset(F, 0, sizeof(F));
	memset(H, 0, sizeof(H));
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (j == needdelete[0])
		{
			needdelete[0] = -1;
			continue;
		} //跳过异常值
		obs = Obs + i;
		if (obs->Prn == 0) continue;
		double Rou = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z));
		H[10 * j + 0] = (X[0] - obs->X) / Rou, H[10 * j + 1] = (X[1] - obs->Y) / Rou, H[10 * j + 2] = (X[2] - obs->Z) / Rou;
		H[10 * j + 3] = H[10 * j + 4] = H[10 * j + 5] = H[10 * j + 8] = H[10 * j + 9] = 0;
		if (obs->Sys == GPS)H[10 * j + 6] = 1, H[10 * j + 7] = 0;
		else if (obs->Sys == BDS)H[10 * j + 6] = 0, H[10 * j + 7] = 1;
		D_delta[MAXCHANNUM * j + j] = obs->error;
		Z[j] = obs->P;
		F[j] = sqrt((X[0] - obs->X) * (X[0] - obs->X) + (X[1] - obs->Y) * (X[1] - obs->Y) + (X[2] - obs->Z) * (X[2] - obs->Z)) + H[10 * j + 6] * X[6] + H[10 * j + 7] * X[7];
		V[j] = Z[j] - F[j];
		j++;
	}
	//结束
	MatrixTranspose(MAXCHANNUM, 10, H, HT);
	MatrixMultiply(10, 10, 10, MAXCHANNUM, D, HT, DHT);
	MatrixMultiply(MAXCHANNUM, 10, 10, MAXCHANNUM, H, DHT, HDHT);
	MatrixAdd(MAXCHANNUM, MAXCHANNUM, HDHT, D_delta, R);
	MatrixInverse(MAXCHANNUM, j, R, RIN);
	MatrixMultiply(10, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, DHT, RIN, K);
	MatrixMultiply(10, MAXCHANNUM, MAXCHANNUM, 1, K, V, KV);
	MatrixMultiply(10, MAXCHANNUM, MAXCHANNUM, 10, K, H, KH);
	MatrixSub(10, 10, I, KH, I_KH); //I_KH = I - KH
	MatrixAdd(10, 1, X, KV, X); //X = X + KV
	MatrixMultiply(10, 10, 10, 10, I_KH, D, D); //D = (I - KH) * D
	XYZ xyz;
	GEOCOOR ref_geo;
	xyz.x = X[0], xyz.y = X[1], xyz.z = X[2];
	ref_geo.latitude = refPos->blh[0] * PI / 180, ref_geo.longitude = refPos->blh[1] * PI / 180, ref_geo.height = refPos->blh[2];
	ToENU(ref_geo, xyz, enu, R_WGS84, E_WGS84);
}