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
	ENUCOOR() : east(0.0), north(0.0), up(0.0) {}
};

struct COMMONTIME   //��ͨ����ʱ��
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
	COMMONTIME() : Year(0), Month(0), Day(0), Hour(0), Minute(0), Second(0.0) {}
};

struct MJDTIME    //�������գ�MJD����1858��11��17����ҹ��ʼ
{
	int Days;
	double FracDay;
	MJDTIME() : Days(0), FracDay(0.0) {}
};

struct GPSTIME   //GPSʱ��
{
	unsigned short Week;
	double SecOfWeek;
	GPSTIME() : Week(0), SecOfWeek(0.0) {}
};

struct SATOBSDATA    //���ǹ۲�ֵ���ݽṹ��
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
			P[i] = 0;
			L[i] = 0;
			D[i] = 0;
			cn0[i] = 0;
			LockTime[i] = 0;
			half[i] = 0;
		}
		Valid = false;
	}
};

struct SATPVT    //����λ���ٶ�ʱ�����ݽṹ��
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;
	SATPVT()
	{
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

struct EPOCHOBSDATA    //�۲�ֵ���ݽṹ��
{
	GPSTIME Time;
	short SatNum;
	SATOBSDATA SatObs[MAXCHANNUM];
	MWGF ComObs[MAXCHANNUM];
	SATPVT SatPVT[MAXCHANNUM];
	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};

struct GPSEPHREC    //�������ݽṹ��
{
	short PRN;
	GNSSsys System;
	GPSTIME TOC, TOE;
	double ClkBias, ClkDrift, ClkDriftRate;
	double IODE, IODC;
	double SqrtA, M0, e, OMEGA, i0, omega;
	double Crs, Cuc, Cus, Cic, Cis, Crc;
	double DeltaN, OMEGADot, iDot;
	int SVHealth;
	double TGD1, TGD2;
	GPSEPHREC()
	{
		PRN = 0;
		IODC = IODE = 0;
		ClkBias = ClkDrift = ClkDriftRate = 0;
		SqrtA = M0 = e = OMEGA = i0 = omega = 0;
		Crs = Cuc = Cus = Cic = Cis = Crc = 0;
		DeltaN = OMEGADot = iDot = 0;
		SVHealth = 0;
		TGD1 = TGD2 = 0;
	}
};

struct POSRES	//��λ������ݽṹ��
{
	GPSTIME Time;
	double Pos[3], Vel[3];
	double PDOP, SigmaPos, SigmaVel;
	double BDS_clockBias, GPS_clockBias;
	int SatNum;
	POSRES()
	{
		//Pos[0] = -2267335.9727, Pos[1] = 5008647.9932, Pos[2] = 3222372.6377;
	    for (int i = 0; i < 3; i++)
		{
			Pos[i] = 0;

			Vel[i] = 0;
		}
		PDOP = SigmaPos = SigmaVel = 0;
		BDS_clockBias = GPS_clockBias = 0;
		//GPS_clockBias = -2.561;
		SatNum = 0;
	}
};