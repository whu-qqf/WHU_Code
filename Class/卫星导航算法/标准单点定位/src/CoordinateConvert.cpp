#include"CoordinateConvert.h"
#include<cmath>

/**
* @file CoordinateConvert.cpp
* @brief 地心地固坐标系与大地坐标系之间的转换函数
* @details
* 大地坐标系（BLH）是指经度、纬度和高程的坐标系统，
* 地心地固坐标系（XYZ）是指以地球中心为原点的三维直角坐标系统。
* 这些函数用于在这两种坐标系统之间进行转换。
* @param geo 大地坐标系坐标
* @param xyz 地心地固坐标系坐标
* @param R 地球半径
* @param E 地球偏心率
*/
void BLHToXYZ(const GEOCOOR& geo, XYZ& xyz, const double R, const double E)
{
	double N = R / sqrt(1 - E * sin(geo.latitude) * sin(geo.latitude));
	xyz.x = (N + geo.height) * cos(geo.latitude) * cos(geo.longitude);
	xyz.y = (N + geo.height) * cos(geo.latitude) * sin(geo.longitude);
	xyz.z = (N * (1 - E) + geo.height) * sin(geo.latitude);
}


/**
* @brief 地心地固坐标系转换为大地坐标系
* @details
* 此函数将地心地固坐标系（XYZ）转换为大地坐标系（BLH）。
* 通过迭代计算纬度和高度，直到达到所需的精度。
* @param xyz 地心地固坐标系坐标
* @param geo 大地坐标系坐标
* @param R 地球半径
* @param E 地球偏心率
*/
void XYZToBLH(const XYZ& xyz, GEOCOOR& geo, const double R, const double E)
{
	geo.longitude = atan2(xyz.y, xyz.x);
	double delta_Z = E * xyz.z;     //迭代初值
	double B, N;
	do {
		B = geo.latitude;
		geo.latitude = atan((xyz.z + delta_Z) / sqrt(xyz.x * xyz.x + xyz.y * xyz.y));
		N = R / sqrt(1 - E * sin(geo.latitude) * sin(geo.latitude));
		delta_Z = N *  E * sin(geo.latitude);
	} while (abs(B-geo.latitude)>1e-9);
	geo.height = sqrt(xyz.x * xyz.x + xyz.y * xyz.y + (xyz.z + delta_Z) * (xyz.z + delta_Z)) - N;
}


/**
* @brief 东北天测站地平坐标系转换函数
* @details
* 此函数将大地坐标系（BLH）或地心地固坐标系（XYZ）转换为测站地平坐标系（ENU）。
* @param origin_xyz 测站地心地固坐标系坐标
* @param target_xyz 目标地心地固坐标系坐标
* @param enu 目标点的测站地平坐标系坐标
* @param R 地球半径
* @param E 地球偏心率
*/
void ToENU(const XYZ& origin_xyz, const XYZ& target_xyz, ENUCOOR& enu, const double R, const double E)
{
	GEOCOOR origin_geo;
	XYZToBLH(origin_xyz, origin_geo, R, E);
	enu.east = -sin(origin_geo.longitude) * (target_xyz.x-origin_xyz.x) + cos(origin_geo.longitude) * (target_xyz.y-origin_xyz.y);
	enu.north = -sin(origin_geo.latitude) * cos(origin_geo.longitude) * (target_xyz.x - origin_xyz.x) - sin(origin_geo.latitude) * sin(origin_geo.longitude) * (target_xyz.y - origin_xyz.y) + cos(origin_geo.latitude) * (target_xyz.z - origin_xyz.z);
	enu.up = cos(origin_geo.latitude) * cos(origin_geo.longitude) * (target_xyz.x - origin_xyz.x) + cos(origin_geo.latitude) * sin(origin_geo.longitude) * (target_xyz.y - origin_xyz.y) + sin(origin_geo.latitude) * (target_xyz.z - origin_xyz.z);
}


/**
* @brief 东北天测站地平坐标系转换函数
* @details
* 此函数将大地坐标系（BLH）或地心地固坐标系（XYZ）转换为测站地平坐标系（ENU）。
* @param origin_geo 测站大地坐标系坐标
* @param target_xyz 目标地心地固坐标系坐标
* @param enu 目标点的测站地平坐标系坐标
* @param R 地球半径
* @param E 地球偏心率
*/
void ToENU(const GEOCOOR& origin_geo, const XYZ& target_xyz, ENUCOOR& enu, const double R, const double E)
{
	XYZ origin_xyz;
	BLHToXYZ(origin_geo, origin_xyz, R, E);
	enu.east = -sin(origin_geo.longitude) * (target_xyz.x - origin_xyz.x) + cos(origin_geo.longitude) * (target_xyz.y - origin_xyz.y);
	enu.north = -sin(origin_geo.latitude) * cos(origin_geo.longitude) * (target_xyz.x - origin_xyz.x) - sin(origin_geo.latitude) * sin(origin_geo.longitude) * (target_xyz.y - origin_xyz.y) + cos(origin_geo.latitude) * (target_xyz.z - origin_xyz.z);
	enu.up = cos(origin_geo.latitude) * cos(origin_geo.longitude) * (target_xyz.x - origin_xyz.x) + cos(origin_geo.latitude) * sin(origin_geo.longitude) * (target_xyz.y - origin_xyz.y) + sin(origin_geo.latitude) * (target_xyz.z - origin_xyz.z);
}


/**
* @brief 卫星高度角和方位角计算函数
* @details
* 此函数计算卫星相对于测站的高度角和方位角。
* @param Xr 测站地心地固坐标系坐标
* @param Xs 卫星地心地固坐标系坐标
* @param Elev 输出的高度角
* @param Azim 输出的方位角
* @param R 地球半径
* @param E 地球偏心率
*/
void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim, const double R, const double E)
{
	XYZ sat, rec;
	for (int i = 0; i < 3; i++)
	{
		sat.xyz[i] = Xs[i];
		rec.xyz[i] = Xr[i];
	}
	GEOCOOR r_geo;
	XYZToBLH(rec, r_geo, R, E);
	double dE = -sin(r_geo.longitude) * (sat.x - rec.x) + cos(r_geo.longitude) * (sat.y - rec.y);
	double dN = -sin(r_geo.latitude) * cos(r_geo.longitude) * (sat.x - rec.x) - sin(r_geo.latitude) * sin(r_geo.longitude) * (sat.y - rec.y) + cos(r_geo.latitude) * (sat.z - rec.z);
	double dU = cos(r_geo.latitude) * cos(r_geo.longitude) * (sat.x - rec.x) + cos(r_geo.latitude) * sin(r_geo.longitude) * (sat.y - rec.y) + sin(r_geo.latitude) * (sat.z - rec.z);
	*Elev = atan(dU / sqrt(dE * dE + dN * dN));
	*Azim = atan2(dE, dN);
}



/**
* @brief 定位误差计算函数
* @details
* 此函数计算测站定位误差，
* 通过比较真实位置和估计位置的差异来计算误差。
* @param X0 真实位置坐标
* @param Xr 估计位置坐标
* @param denu 输出的定位误差（东、北、天）
* @param R 地球半径
* @param E 地球偏心率
*/
void CompEnudPos(const double X0[], const double Xr[], double denu[], const double R, const double E)  //定位误差计算函数
{
	XYZ tru, est;
	for (int i = 0; i < 3; i++)
	{
		tru.xyz[i] = X0[i];
		est.xyz[i] = Xr[i];
	}
	GEOCOOR tru_geo;
	XYZToBLH(tru, tru_geo, R, E);
	double dE = -sin(tru_geo.longitude) * (est.x - tru.x) + cos(tru_geo.longitude) * (est.y - tru.y);
	double dN = -sin(tru_geo.latitude) * cos(tru_geo.longitude) * (est.x - tru.x) - sin(tru_geo.latitude) * sin(tru_geo.longitude) * (est.y - tru.y) + cos(tru_geo.latitude) * (est.z - tru.z);
	double dU = cos(tru_geo.latitude) * cos(tru_geo.longitude) * (est.x - tru.x) + cos(tru_geo.latitude) * sin(tru_geo.longitude) * (est.y - tru.y) + sin(tru_geo.latitude) * (est.z - tru.z);
	denu[0] = dE;
	denu[1] = dN;
	denu[2] = dU;
}