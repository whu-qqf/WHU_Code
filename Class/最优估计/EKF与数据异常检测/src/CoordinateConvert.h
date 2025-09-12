#pragma once
#include"const.h"
#include"struct.h"


void BLHToXYZ(const GEOCOOR& geo, XYZ& xyz,const double R,const double E);   //�������ϵת��Ϊ���ĵع�����ϵ
void XYZToBLH(const XYZ& xyz, GEOCOOR& geo, const double R, const double E);   //���ĵع�����ϵת��Ϊ�������ϵ

void ToENU(const XYZ& origin_xyz, const XYZ& target_xyz,  ENUCOOR& enu, const double R, const double E);   //���ĵع�����ϵת��Ϊ��վ��ƽ����ϵ
void ToENU(const GEOCOOR& origin_geo, const XYZ& target_xyz, ENUCOOR& enu, const double R, const double E);   //�������ϵת��Ϊ��վ��ƽ����ϵ

void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim, const double R, const double E); //���Ǹ߶ȽǷ�λ�Ǽ��㺯��
void CompEnudPos(const double X0[], const double Xr[], double denu[], const double R, const double E);  //��λ�����㺯��