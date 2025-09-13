#pragma once
class Coordinate
{
public:
	void BLH2XYZ(double*m, double*n);    //声明从BLH转换到XYZ函数
	void XYZ2BLH(double*c, double*d);    //声明从XYZ转换到BLH函数
	void XYZ2NED(double*p, double*q,double*w);    //声明从XYZ转换到NED（站心坐标系）的函数
	void NED2FRD(double*x, double *t, double*a, double*g);     //声明从NED转换到FRD（载体坐标系）的函数
};