#pragma once
#include<vector>
using namespace std;
class Coordinate
{
public:
	void BLH2XYZ(double*m, double*n);    //声明从BLH转换到XYZ函数
	void XYZ2BLH(double*c, double*d);    //声明从XYZ转换到BLH函数
	//地心地固坐标系XYZ到大地坐标系BLH
	vector<vector<double>> XYZ2BLH(vector<vector<double>>a);
	void XYZ2NED(double*p, double*q,double*w);    //声明从XYZ转换到NED（北东地站心坐标系）的函数
	//地心地固坐标系XYZ到东北天站心坐标系
	vector<vector<double>> XYZ2ENU(vector<vector<double>>a, vector<vector<double>>b);
	void NED2FRD(double*x, double *t, double*a, double*g);     //声明从NED转换到FRD（载体坐标系）的函数
};