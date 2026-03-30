#pragma once
#include<vector>
#include<cstring>
#include"const.h"
enum GNSSsys { GPS = 1, BDS };

using namespace std;

union XYZ    //地心地固坐标系
{
	struct {
		double x;
		double y;
		double z;
	};
	double xyz[3];
};

union GEOCOOR               //大地坐标系
{
	struct {
		double longitude;   //经度
		double latitude;    //纬度
		double height;      //高程
	};
	double blh[3];
	GEOCOOR() : longitude(0.0), latitude(0.0), height(0.0) {}
};

union ENUCOOR           //测站地平坐标系
{
	struct {
		double east;    //东
		double north;   //北
		double up;      //天
	};
	double enu[3];
	ENUCOOR() : east(0.0), north(0.0), up(0.0) {}
};

struct COMMONTIME   //普通公历时间
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
	COMMONTIME() : Year(0), Month(0), Day(0), Hour(0), Minute(0), Second(0.0) {}
};

struct MJDTIME    //简化儒略日（MJD）从1858年11月17日子夜开始
{
	int Days;
	double FracDay;
	MJDTIME() : Days(0), FracDay(0.0) {}
};

struct GPSTIME   //GPS时间
{
	unsigned short Week;
	double SecOfWeek;
	GPSTIME() : Week(0), SecOfWeek(0.0) {}
};


struct POSRES	//定位结果数据结构体
{
	GPSTIME Time;
	double Pos[3], Vel[3];
	double PDOP, SigmaPos, SigmaVel;
	double BDS_clockBias, GPS_clockBias;
	int SatNum;
	POSRES()
	{
	    for (int i = 0; i < 3; i++)
		{
			Pos[i] = 0;

			Vel[i] = 0;
		}
		PDOP = SigmaPos = SigmaVel = 0;
		BDS_clockBias = GPS_clockBias = 0;
		SatNum = 0;
	}
};


struct IMU_DATA
{
	vector<int>week;
	vector<double>secofweek;
	vector<double>x_acc;
	vector<double>y_acc;
	vector<double>z_acc;
	vector<double>x_gyo;
	vector<double>y_gyo;
	vector<double>z_gyo;
	vector<double>* ACC_DATA[3];
	vector<double>* GYO_DATA[3];
	IMU_DATA()
	{
		ACC_DATA[0] = &x_acc;
		ACC_DATA[1] = &y_acc;
		ACC_DATA[2] = &z_acc;

		GYO_DATA[0] = &x_gyo;
		GYO_DATA[1] = &y_gyo;
		GYO_DATA[2] = &z_gyo;
	}
};

struct STATIC_IMU_DATA
{
	short week;         /*GPS周*/
	double secofweek;   /*周内秒*/
	GPSTIME gpsTime;    /*GPS时间*/
	double x_acc;       /*X轴加速度*/
	double y_acc;       /*Y轴加速度*/
	double z_acc;       /*Z轴加速度*/
	double x_gyo;       /*X轴角速度*/
	double y_gyo;       /*Y轴角速度*/
	double z_gyo;       /*Z轴角速度*/
	STATIC_IMU_DATA()
	{
		week = 0;
		secofweek = x_acc = y_acc = z_acc = x_gyo = y_gyo = z_gyo = 0.0;
	}
};

struct IMU_ERROR
{
	double M[3 * 4], Gyo[3 * 4];
	IMU_ERROR()
	{
		memset(M, 0, 12 * sizeof(double));
		memset(Gyo, 0, 12 * sizeof(double));
	}
};

struct IMU_POS
{
	GEOCOOR geo_2;
	GEOCOOR geo_1;
	GEOCOOR geo_0;
	double v_2[3];
	double v_1[3];
	double v_0[3];
	double q_1[4];
	double q_0[4];
	IMU_POS()
	{
		for (int i = 0; i < 3; i++)v_2[i] = v_1[i] = v_0[i] = q_1[i] = q_0[i] = 0.0;
		q_1[3] = q_0[3] = 0.0;
	}
};


struct EulerAngle
{
	double roll;    //横滚角X
	double pitch;    //俯仰角Y
	double yaw;    //航向角Z
	EulerAngle()
	{
		roll = pitch = yaw = 0.0;
	}
};

struct IMUResult
{
	GPSTIME gpsTime;
	GEOCOOR geo;
	double velocity[3];
	EulerAngle attitude;
	IMUResult()
	{
		velocity[0] = velocity[1] = velocity[2] = 0.0;
	}
};