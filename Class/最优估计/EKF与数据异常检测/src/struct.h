#pragma once
#include"const.h"

enum GNSSsys { GPS = 1, BDS };

union XYZ    //���ĵع�����ϵ
{
	struct {
		double x;
		double y;
		double z;
	};
	double xyz[3];
};

union GEOCOOR    //�������ϵ
{
	struct {
		double longitude;   //����
		double latitude;   //γ��
		double height;   //�߳�
	};
	double blh[3];
	GEOCOOR() : longitude(0.0), latitude(0.0), height(0.0) {}
};

union ENUCOOR    //��վ��ƽ����ϵ
{
	struct {
		double east;   //��
		double north;   //��
		double up;   //��
	};
	double enu[3];
};

struct ReferencePos
{
	int t;
	double blh[3];  //xyz����
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

