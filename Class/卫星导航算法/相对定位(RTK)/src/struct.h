#pragma once

#include<cstring>
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

struct SATOBSDATA    //卫星观测值数据结构体
{
	short Prn;
	GNSSsys System;
	double P[2], L[2], D[2];
	double cn0[2], LockTime[2];
	unsigned char half[2];
	bool Valid;
	SATOBSDATA()
	{
		Prn = 0;
		System = GPS;
		for (int i = 0; i < 2; i++)
		{
			P[i]        = 0;
			L[i]        = 0;
			D[i]        = 0;
			cn0[i]      = 0;
			LockTime[i] = 0;
			half[i]     = 0;
		}
		Valid = false;
	}
};

struct SATPVT    //卫星位置速度时间数据结构体
{
	GNSSsys System;
	short Prn;
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;
	SATPVT()
	{
		System = GPS;
		Prn = 0;
		for (int i = 0; i < 3; i++)
		{
			SatPos[i] = 0;
			SatVel[i] = 0;
		}
		Elevation = PI / 2;
		Azimuth = 0;
		SatClkOft = SatClkSft = 0;
		Tgd1 = Tgd2 = TropCorr = 0;
		Valid = false;
	}
};

struct MWGF
{
	short Prn;
	GNSSsys Sys;
	double MW;
	double GF;
	double PIF;
	int n;
	MWGF()
	{
		Prn = 0;
		Sys = GPS;
		MW = 0;
		GF = 0;
		PIF = 0;
		n = 0;
	}
};

struct EPOCHOBSDATA    //观测值数据结构体
{
	GPSTIME Time;
	short SatNum;
	SATOBSDATA SatObs[MAXCHANNUM];   //卫星观测值
	MWGF ComObs[MAXCHANNUM];     //组合观测值数据
	SATPVT SatPVT[MAXCHANNUM];    //卫星位置速度时间数据
	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};

struct GPSEPHREC    //星历数据结构体
{
	short   PRN;
	GNSSsys System;
	GPSTIME TOC, TOE;
	double  ClkBias, ClkDrift, ClkDriftRate;
	double  IODE, IODC;
	double  SqrtA, M0, e, OMEGA, i0, omega;
	double  Crs, Cuc, Cus, Cic, Cis, Crc;
	double  DeltaN, OMEGADot, iDot;
	int     SVHealth;
	double  TGD1, TGD2;
	GPSEPHREC()
	{
		PRN      = 0;
		IODC     = IODE     = 0;
		ClkBias  = ClkDrift = ClkDriftRate = 0;
		SqrtA    = M0       = e            = OMEGA = i0  = omega = 0;
		Crs      = Cuc      = Cus          = Cic   = Cis = Crc   = 0;
		DeltaN   = OMEGADot = iDot         = 0;
		SVHealth = 0;
		TGD1     = TGD2     = 0;
	}
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

struct RAWDATA{            /*RTK原始数据*/
	EPOCHOBSDATA BaseObs;  /*基站原始数据*/
	EPOCHOBSDATA RoverObs; /*流动站原始数据*/
	GPSEPHREC    GPSEph[MAXGPSNUM], BDSEph[MAXBDSNUM];   /*星历数据*/
	//GPSEPHREC    roverGPSEph[MAXGPSNUM], roverBDSEph[MAXBDSNUM];
	POSRES       bestPos;      /*基站定位结果数据*/
	POSRES       bestRoverPos;
};


struct SDSATOBS{      /*站间单差观测值*/
	short Prn;    /*卫星PNR*/
	GNSSsys System;    /*卫星系统*/
	double dP[2], dL[2];   /*伪距单差，相位单差*/
	int iBas, iRov;   /*基准站和流动站的观测值索引号*/
	bool Valid;    /*观测值是否有效*/
	SDSATOBS()
	{
		Prn = iBas = iRov = 0;
		System = GPS;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = false;
	}
};


struct SDEPOCHOBS{      /*每个历元的单差观测数据*/
	GPSTIME  Time;
	short    SatNum;
	SDSATOBS SdSatObs[MAXCHANNUM];
	MWGF     SdCObs[MAXCHANNUM];   /*单差组合观测值*/
	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

struct DDCOBS{
	int RefPrn[2],RefIndex[2];         /*参考星卫星号与存储位置，0=GPS; 1=BDS*/
	int DDSatNums[2];                  /*可用的双差卫星数量*/
	double dPos[3];                   /* 基线向量*/
	bool bFixed;                      /* true为固定，false为未固定*/
	double Float_Ambi[(MAXCHANNUM - 2) * 2];     /*浮点模糊度*/
	double Qnn[(3 + (MAXCHANNUM - 2) * 2) * (3 + (MAXCHANNUM - 2) * 2)] = { 0.0 };    /*模糊度协因数阵*/
	DDCOBS()
	{
		for (int i = 0; i < 2; i++) 
		{
			DDSatNums[i] = 0;    /*各卫星系统的双差数量*/
			RefIndex[i] = RefPrn[i] = -1;
		}
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		std::memset(Float_Ambi, 0, sizeof(Float_Ambi));
		std::memset(Qnn, 0, sizeof(Qnn));
		bFixed = false;
	}
};

struct EKFRTK { 
	GPSTIME Time;        /*GPS时*/
	double X[3 + (MAXCHANNUM - 2) * 2];     /*状态矩阵*/
	double P[(3 + (MAXCHANNUM - 2) * 2) * (3 + (MAXCHANNUM - 2) * 2)];    /*状态方差*/
	double Ambi[(MAXCHANNUM - 2) * 2];    /*模糊度矩阵*/
	int DDSatPrn_GPS[MAXGPSNUM];    /*参与解算的GPS卫星号*/
	int DDSatPrn_BDS[MAXBDSNUM];    /*参与解算的BDS卫星号*/
	bool isInited;
	EKFRTK()
	{
		std::memset(X, 0, sizeof(X));
		std::memset(P, 0, sizeof(P));
		std::memset(Ambi, 0, sizeof(Ambi));
		std::memset(DDSatPrn_GPS, 0, sizeof(DDSatPrn_GPS));
		std::memset(DDSatPrn_BDS, 0, sizeof(DDSatPrn_BDS));
		isInited = false;
	}
};