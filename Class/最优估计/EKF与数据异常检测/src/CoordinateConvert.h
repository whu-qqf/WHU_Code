#pragma once
#include"const.h"
#include"struct.h"


void BLHToXYZ(const GEOCOOR& geo, XYZ& xyz,const double R,const double E);   //大地坐标系转换为地心地固坐标系
void XYZToBLH(const XYZ& xyz, GEOCOOR& geo, const double R, const double E);   //地心地固坐标系转换为大地坐标系

void ToENU(const XYZ& origin_xyz, const XYZ& target_xyz,  ENUCOOR& enu, const double R, const double E);   //地心地固坐标系转换为测站地平坐标系
void ToENU(const GEOCOOR& origin_geo, const XYZ& target_xyz, ENUCOOR& enu, const double R, const double E);   //大地坐标系转换为测站地平坐标系

void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim, const double R, const double E); //卫星高度角方位角计算函数
void CompEnudPos(const double X0[], const double Xr[], double denu[], const double R, const double E);  //定位误差计算函数