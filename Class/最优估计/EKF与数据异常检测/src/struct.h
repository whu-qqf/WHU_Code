#pragma once
#include"const.h"

enum GNSSsys { GPS = 1, BDS };

union XYZ    //地心地固坐标系
{
	struct {
		double x;
		double y;
		double z;
	};
	double xyz[3];
};

union GEOCOOR    //大地坐标系
{
	struct {
		double longitude;   //经度
		double latitude;   //纬度
		double height;   //高程
	};
	double blh[3];
	GEOCOOR() : longitude(0.0), latitude(0.0), height(0.0) {}
};

union ENUCOOR    //测站地平坐标系
{
	struct {
		double east;   //东
		double north;   //北
		double up;   //天
	};
	double enu[3];
};

struct ReferencePos
{
	int t;
	double blh[3];  //xyz坐标
};

struct EKFOBS
{
	GNSSsys Sys;
	short Prn;
	double X;
	double Y;
	double Z;
	double P;
	double error;
	EKFOBS()
	{
		Sys = GPS;
		Prn = 0;
		X = Y = Z = P = 0.0;
		error = 0.0;
	}
};

