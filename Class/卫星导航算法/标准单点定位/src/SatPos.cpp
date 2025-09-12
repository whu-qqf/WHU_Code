#include"SatPos.h"
#include"const.h"
#include<iostream>
#include<cmath>
#include"Error.h"
#include"Matrix.h"
#include"CoordinateConvert.h"


using namespace std;

/**
* @brief 计算卫星钟差和钟速
* @param Prn 卫星编号
* @param Sys 卫星系统
* @param t 信号发射时刻
* @param GPSEph GPS卫星星历
* @param BDSEph BDS卫星星历
* @param Mid 卫星PVT结构体指针
* @return true:计算成功; false:计算失败
*/
bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid)
{
	double dt, LimT = 7500.0;
	GPSTIME CurT;
	GPSEPHREC* eph;
	CurT = *t;
	if (Sys == GPS) eph = GPSEph + Prn - 1;
	else if (Sys == BDS) {
		eph = BDSEph + Prn - 1;
		CurT.Week -= 1356;
		CurT.SecOfWeek -= 14;
		LimT = 3900.0;
	}
	else return false;
	dt = (CurT.Week - eph->TOC.Week) * 604800.0 + CurT.SecOfWeek - eph->TOC.SecOfWeek;
	if (fabs(dt) > LimT || eph->SVHealth != 0) return false;
	Mid->SatClkOft = eph->ClkBias + eph->ClkDrift * dt + eph->ClkDriftRate * pow(dt, 2);
	Mid->SatClkSft = eph->ClkDrift + 2.0 * eph->ClkDriftRate * dt;
	return true;
}

/**
* @brief 计算GPS卫星位置、速度和钟差
* @param Prn 卫星编号
* @param t 信号发射时刻
* @param GPSEph GPS卫星星历
* @param Mid 卫星PVT结构体指针
* @return 0:计算成功; 非0:计算失败
*/
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* GPSEph, SATPVT* Mid)
{
	double A, n0, tk, n, Mk, Ek, Ek_0, vk, Phik, delta_uk, delta_rk, delta_ik, uk, rk, ik, OMEGAk, xk, yk;
	double EkDot, PhikDot, ukDot, rkDot, ikDot, OMEGAkDot, xkDot, ykDot;
	double X[3];
	double delta_tr, delta_trDot;
	int i = 0;
	A = GPSEph->SqrtA * GPSEph->SqrtA;
	n0 = sqrt(GM_GPS) / pow(GPSEph->SqrtA, 3);
	tk = t->SecOfWeek - GPSEph->TOE.SecOfWeek;
	n = GPSEph->DeltaN + n0;
	Mk = GPSEph->M0 + n * tk;
	Ek = Mk;
	do
	{
		i++;
		Ek_0 = Ek;
		Ek = Mk + GPSEph->e * sin(Ek);
		if (i > 10) break;
	} while (fabs(Ek - Ek_0) > 1E-12);
	vk = atan2(sqrt(1 - GPSEph->e * GPSEph->e) * sin(Ek), cos(Ek) - GPSEph->e);
	Phik = GPSEph->omega + vk;
	delta_uk = GPSEph->Cuc * cos(2 * Phik) + GPSEph->Cus * sin(2 * Phik);
	delta_rk = GPSEph->Crs * sin(2 * Phik) + GPSEph->Crc * cos(2 * Phik);
	delta_ik = GPSEph->Cic * cos(2 * Phik) + GPSEph->Cis * sin(2 * Phik);
	uk = Phik + delta_uk;
	rk = A * (1 - GPSEph->e * cos(Ek)) + delta_rk;
	ik = GPSEph->i0 + delta_ik + GPSEph->iDot * tk;
	OMEGAk = GPSEph->OMEGA + (GPSEph->OMEGADot - OMEGAE_GPS) * tk - OMEGAE_GPS * GPSEph->TOE.SecOfWeek;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	Mid->SatPos[0] = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
	Mid->SatPos[1] = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
	Mid->SatPos[2] = yk * sin(ik);
	delta_tr = F * GPSEph->e * sin(Ek) * GPSEph->SqrtA;
	Mid->SatClkOft =GPSEph->ClkBias + GPSEph->ClkDrift * (t->SecOfWeek - GPSEph->TOC.SecOfWeek) + GPSEph->ClkDriftRate * (t->SecOfWeek - GPSEph->TOC.SecOfWeek) * (t->SecOfWeek - GPSEph->TOC.SecOfWeek)+delta_tr;
	delta_tr;
	EkDot = n / (1 - GPSEph->e * cos(Ek));
	PhikDot = sqrt(1 - GPSEph->e * GPSEph->e) * EkDot / (1 - GPSEph->e * cos(Ek));
	ukDot = PhikDot + 2 * (-GPSEph->Cuc * sin(2 * Phik) + GPSEph->Cus * cos(2 * Phik)) * PhikDot;
	rkDot = A * GPSEph->e * sin(Ek) * EkDot + 2 * (GPSEph->Crs * cos(2 * Phik) - GPSEph->Crc * sin(2 * Phik)) * PhikDot;
	ikDot = GPSEph->iDot + 2 * (-GPSEph->Cic * sin(2 * Phik) + GPSEph->Cis * cos(2 * Phik)) * PhikDot;
	OMEGAkDot = GPSEph->OMEGADot - OMEGAE_GPS;	
	xkDot = rkDot * cos(uk) - rk * sin(uk) * ukDot;
	ykDot = rkDot * sin(uk) + rk * cos(uk) * ukDot;
	double RDot[12] = {  cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik),
		sin(OMEGAk), cos(OMEGAk) * cos(ik), xk * cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik),
		0, sin(ik), 0, yk * cos(ik)} ;
	double V[4] = { xkDot,ykDot,OMEGAkDot,ikDot };
	MatrixMultiply(3, 4, 4, 1, RDot, V, X);
	Mid->SatVel[0] = X[0];
	Mid->SatVel[1] = X[1];
	Mid->SatVel[2] = X[2];
	delta_trDot = F * EkDot * cos(Ek) * GPSEph->SqrtA * GPSEph->e;    //相对论改正钟差
	Mid->SatClkSft = GPSEph->ClkDrift + 2 * GPSEph->ClkDriftRate * (t->SecOfWeek - GPSEph->TOC.SecOfWeek)+delta_trDot;
	Mid->Tgd1 = GPSEph->TGD1;
	return 0;
}

/**
* @brief 计算BDS卫星位置、速度和钟差
* @param Prn 卫星编号
* @param t 信号发射时刻
* @param BDSEph BDS卫星星历
* @param Mid 卫星PVT结构体指针
* @return 0:计算成功; 非0:计算失败
*/
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* BDSEph, SATPVT* Mid)
{
	double A, n0, tk, n, Mk, Ek, Ek_0, vk, Phik, delta_uk, delta_rk, delta_ik, uk, rk, ik, OMEGAk, xk, yk;
	double EkDot, PhikDot, ukDot, rkDot, ikDot, OMEGAkDot, xkDot, ykDot;
	double xgk, ygk, zgk;
	double delta_tr, delta_trDot;
	double X[3];
	int i = 0;
	GPSTIME bdt;
	bdt.SecOfWeek = t->SecOfWeek - GPST_BDT_SEC;
	bdt.Week = t->Week - GPST_BDT_WEEK;
	A = BDSEph->SqrtA * BDSEph->SqrtA;
	n0 = sqrt(GM_BDS) / pow(BDSEph->SqrtA,3);

	tk = bdt.SecOfWeek - BDSEph->TOE.SecOfWeek;
	n = BDSEph->DeltaN + n0;
	Mk = BDSEph->M0 + n * tk;
	Ek = Mk;
	do
	{
		i++;
		Ek_0 = Ek;
		Ek = Mk + BDSEph->e * sin(Ek);
		if (i > 10) break;
	} while (fabs(Ek - Ek_0) > 1E-12);
	vk = atan2(sqrt(1 - BDSEph->e * BDSEph->e) * sin(Ek), cos(Ek) - BDSEph->e);
	Phik = BDSEph->omega + vk;
	delta_uk = BDSEph->Cuc * cos(2 * Phik) + BDSEph->Cus * sin(2 * Phik);
	delta_rk = BDSEph->Crs * sin(2 * Phik) + BDSEph->Crc * cos(2 * Phik);
	delta_ik = BDSEph->Cic * cos(2 * Phik) + BDSEph->Cis * sin(2 * Phik);
	uk = Phik + delta_uk;
	rk = A * (1 - BDSEph->e * cos(Ek)) + delta_rk;
	ik = BDSEph->i0 + delta_ik + BDSEph->iDot * tk;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	EkDot = n / (1 - BDSEph->e * cos(Ek));
	PhikDot = sqrt(1 - BDSEph->e * BDSEph->e) * EkDot / (1 - BDSEph->e * cos(Ek));
	ukDot = PhikDot + 2 * (-BDSEph->Cuc * sin(2 * Phik) + BDSEph->Cus * cos(2 * Phik)) * PhikDot;
	rkDot = A * BDSEph->e * sin(Ek) * EkDot + 2 * (BDSEph->Crs * cos(2 * Phik) - BDSEph->Crc * sin(2 * Phik)) * PhikDot;
	ikDot = BDSEph->iDot + 2 * (-BDSEph->Cic * sin(2 * Phik) + BDSEph->Cis * cos(2 * Phik)) * PhikDot;
	xkDot = rkDot * cos(uk) - rk * sin(uk) * ukDot;
	ykDot = rkDot * sin(uk) + rk * cos(uk) * ukDot;
	if (Prn <= 5 || (Prn >= 59 && Prn <= 63))    //GEO卫星
	{
		OMEGAk = BDSEph->OMEGA + (BDSEph->OMEGADot) * tk - OMEGAE_BDS * BDSEph->TOE.SecOfWeek;
		xgk = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
		ygk = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
		zgk = yk * sin(ik);
		double Xgk[3] = { xgk,ygk,zgk };
		double Rx[9] = { 1,0,0,0,cos(-PI / 36),sin(-PI / 36),0,-sin(-PI / 36),cos(PI / 36) };
		double Rz[9] = { cos(OMEGAE_BDS * tk),sin(OMEGAE_BDS * tk),0,-sin(OMEGAE_BDS * tk),cos(OMEGAE_BDS * tk),0,0,0,1 };
		double Rotate[9] = { 0,0,0,0,0,0,0,0,0 };
		double Pos[3];
		MatrixMultiply(3, 3, 3, 3, Rz, Rx, Rotate);
		MatrixMultiply(3, 3, 3, 1, Rotate, Xgk, Pos);
		Mid->SatPos[0] = Pos[0];
		Mid->SatPos[1] = Pos[1];
		Mid->SatPos[2] = Pos[2];
		OMEGAkDot = BDSEph->OMEGADot;
		double RDot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik),
			sin(OMEGAk), cos(OMEGAk)* cos(ik), xk* cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik),
		0, sin(ik), 0, yk* cos(ik) };
		double V[4] = { xkDot,ykDot,OMEGAkDot,ikDot };
		double XgkDot[3];
		MatrixMultiply(3, 4, 4, 1, RDot, V, XgkDot);
		double RzDot[9] = { -OMEGAE_BDS * sin(OMEGAE_BDS * tk),OMEGAE_BDS * cos(OMEGAE_BDS * tk),0,-OMEGAE_BDS * cos(OMEGAE_BDS * tk),-OMEGAE_BDS * sin(OMEGAE_BDS * tk),0,0,0,0 };
		double Vel[3], M1[3], RzDotMRx[9], M2[3];
		MatrixMultiply(3, 3, 3, 1, Rotate, XgkDot, M1);
		MatrixMultiply(3, 3, 3, 3, RzDot, Rx, RzDotMRx);
		MatrixMultiply(3, 3, 3, 1, RzDotMRx, Xgk, M2);
		MatrixAdd(3, 1, M1, M2, Vel);
		Mid->SatVel[0] = Vel[0];
		Mid->SatVel[1] = Vel[1];
		Mid->SatVel[2] = Vel[2];

	}
	else 
	{
		OMEGAk = BDSEph->OMEGA + (BDSEph->OMEGADot - OMEGAE_BDS) * tk - OMEGAE_BDS * BDSEph->TOE.SecOfWeek;
		Mid->SatPos[0] = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
		Mid->SatPos[1] = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
		Mid->SatPos[2] = yk * sin(ik);
		OMEGAkDot = BDSEph->OMEGADot - OMEGAE_BDS;
		double RDot[12] = {  cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik),
		sin(OMEGAk), cos(OMEGAk) * cos(ik), xk * cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik),
		0, sin(ik), 0, yk * cos(ik) };
		double V[4] = { xkDot,ykDot,OMEGAkDot,ikDot} ;
		MatrixMultiply(3, 4, 4, 1, RDot, V, X);
		Mid->SatVel[0] = X[0];
		Mid->SatVel[1] = X[1];
		Mid->SatVel[2] = X[2];
	}
	delta_tr = F * BDSEph->e * sin(Ek) * BDSEph->SqrtA;    //相对论改正钟差
	delta_trDot = F * EkDot * cos(Ek) * BDSEph->SqrtA * BDSEph->e;
	Mid->SatClkOft = BDSEph->ClkBias + BDSEph->ClkDrift * (bdt.SecOfWeek - BDSEph->TOC.SecOfWeek) + BDSEph->ClkDriftRate * (bdt.SecOfWeek - BDSEph->TOC.SecOfWeek) * (bdt.SecOfWeek - BDSEph->TOC.SecOfWeek)+delta_tr;
	Mid->SatClkSft = BDSEph->ClkDrift + 2 * BDSEph->ClkDriftRate * (bdt.SecOfWeek - BDSEph->TOC.SecOfWeek) + delta_trDot;
	Mid->Tgd1 = BDSEph->TGD1;
	Mid->Tgd2 = BDSEph->TGD2;
	return 0;
}

/**
* @brief 地球自转改正
* @param UserPos 用户位置（ECEF坐标）
* @param Eph 卫星星历
* @param Mid 卫星PVT结构体指针
*/ 
int EarthRotate(double UserPos[3], const GPSEPHREC* Eph, SATPVT* Mid)
{
	double delta_t, omegae, R, E, el;
	GEOCOOR UserGeo;
	XYZ xyz;
	if(UserPos[0] == 0 && UserPos[1] == 0 && UserPos[2] == 0)
	{
		return 1;
	}
	for (int i = 0; i < 3; i++)
	{
		xyz.xyz[i] = UserPos[i];
	}
	if (Eph->System == GPS) omegae = OMEGAE_GPS, R = R_CGS2K, E = E_CGS2K;
	else omegae = OMEGAE_BDS, R = R_WGS84, E = E_WGS84;
	XYZToBLH(xyz, UserGeo, R, E);
	delta_t = sqrt((Mid->SatPos[0]-xyz.x) * (Mid->SatPos[0] - xyz.x) + (Mid->SatPos[1] - xyz.y) * (Mid->SatPos[1] - xyz.y) + (Mid->SatPos[2] - xyz.z) * (Mid->SatPos[2] - xyz.z))/C_Light;
	double Rz[9] = {cos(omegae * delta_t),sin(omegae * delta_t),0,-sin(omegae * delta_t),cos(omegae * delta_t),0,0,0,1 };
	MatrixMultiply(3, 3, 3, 1, Rz, Mid->SatPos,Mid->SatPos);
	MatrixMultiply(3, 3, 3, 1, Rz, Mid->SatVel, Mid->SatVel);
	CompSatElAz(xyz.xyz, Mid->SatPos, &(Mid->Elevation), &(Mid->Azimuth), R, E);
	el = Mid->Elevation * 180 / PI;
	Mid->TropCorr = Hopfield(UserGeo.height, el);    //计算对流层改正
	return 0;
}