#include"CoordinateConvert.h"
#include<cmath>

/**
* @file CoordinateConvert.cpp
* @brief ���ĵع�����ϵ��������ϵ֮���ת������
* @details
* �������ϵ��BLH����ָ���ȡ�γ�Ⱥ͸̵߳�����ϵͳ��
* ���ĵع�����ϵ��XYZ����ָ�Ե�������Ϊԭ�����άֱ������ϵͳ��
* ��Щ��������������������ϵͳ֮�����ת����
* @param geo �������ϵ����
* @param xyz ���ĵع�����ϵ����
* @param R ����뾶
* @param E ����ƫ����
*/
void BLHToXYZ(const GEOCOOR& geo, XYZ& xyz, const double R, const double E)
{
	double N = R / sqrt(1 - E * sin(geo.latitude) * sin(geo.latitude));
	xyz.x = (N + geo.height) * cos(geo.latitude) * cos(geo.longitude);
	xyz.y = (N + geo.height) * cos(geo.latitude) * sin(geo.longitude);
	xyz.z = (N * (1 - E) + geo.height) * sin(geo.latitude);
}


/**
* @brief ���ĵع�����ϵת��Ϊ�������ϵ
* @details
* �˺��������ĵع�����ϵ��XYZ��ת��Ϊ�������ϵ��BLH����
* ͨ����������γ�Ⱥ͸߶ȣ�ֱ���ﵽ����ľ��ȡ�
* @param xyz ���ĵع�����ϵ����
* @param geo �������ϵ����
* @param R ����뾶
* @param E ����ƫ����
*/
void XYZToBLH(const XYZ& xyz, GEOCOOR& geo, const double R, const double E)
{
	geo.longitude = atan2(xyz.y, xyz.x);
	double delta_Z = E * xyz.z;     //������ֵ
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
* @brief �������վ��ƽ����ϵת������
* @details
* �˺������������ϵ��BLH������ĵع�����ϵ��XYZ��ת��Ϊ��վ��ƽ����ϵ��ENU����
* @param origin_xyz ��վ���ĵع�����ϵ����
* @param target_xyz Ŀ����ĵع�����ϵ����
* @param enu Ŀ���Ĳ�վ��ƽ����ϵ����
* @param R ����뾶
* @param E ����ƫ����
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
* @brief �������վ��ƽ����ϵת������
* @details
* �˺������������ϵ��BLH������ĵع�����ϵ��XYZ��ת��Ϊ��վ��ƽ����ϵ��ENU����
* @param origin_geo ��վ�������ϵ����
* @param target_xyz Ŀ����ĵع�����ϵ����
* @param enu Ŀ���Ĳ�վ��ƽ����ϵ����
* @param R ����뾶
* @param E ����ƫ����
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
* @brief ���Ǹ߶ȽǺͷ�λ�Ǽ��㺯��
* @details
* �˺���������������ڲ�վ�ĸ߶ȽǺͷ�λ�ǡ�
* @param Xr ��վ���ĵع�����ϵ����
* @param Xs ���ǵ��ĵع�����ϵ����
* @param Elev ����ĸ߶Ƚ�
* @param Azim ����ķ�λ��
* @param R ����뾶
* @param E ����ƫ����
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
* @brief ��λ�����㺯��
* @details
* �˺��������վ��λ��
* ͨ���Ƚ���ʵλ�ú͹���λ�õĲ�����������
* @param X0 ��ʵλ������
* @param Xr ����λ������
* @param denu ����Ķ�λ�����������죩
* @param R ����뾶
* @param E ����ƫ����
*/
void CompEnudPos(const double X0[], const double Xr[], double denu[], const double R, const double E)  //��λ�����㺯��
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