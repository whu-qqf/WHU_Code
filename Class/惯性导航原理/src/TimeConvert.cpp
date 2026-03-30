#include<iostream>
#include<cmath>
#include"TimeConvert.h"

/**
* @brief 通用时转化为简化儒略日
* @details
* 此函数将通用时转化为简化儒略日
* @param common 通用时
* @param mjd 简化儒略日
*/
void CommonTimetoMjdTime(const COMMONTIME& common, MJDTIME& mjd)
{
	short year = common.Year;
	unsigned short month = common.Month;
	if (common.Month <= 2)
	{
		year--;
		month += 12;
	}
	double UT = common.Hour + common.Minute / 60.0 + common.Second / 3600.0;
	double JD = int(365.25 * year) + int(30.6001 * (month + 1)) + common.Day + UT / 24.0 + 1720981.5;
	mjd.Days = int(JD - 2400000.5);
	mjd.FracDay = UT / 24.0;
}

/**
* @brief 简化儒略日转化为通用时
* @details
* 此函数将简化儒略日转化为通用时
* @param mjd 简化儒略日
* @param common 通用时
*/
void MjdTimeToCommonTime(const MJDTIME& mjd, COMMONTIME& common)
{
	double JD = mjd.Days + 2400000.5 + mjd.FracDay;
	int a = int(JD+0.5);
	int b = a + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b - d) / 30.6001);
	common.Day = b - d - int(30.6001 * e);
	common.Month = e - 1 - 12 * int(e / 14.0);
	common.Year = c - 4715 - int((7 + common.Month) / 10.0);
	common.Hour = int(mjd.FracDay * 24);
	common.Minute = int((mjd.FracDay * 24 - common.Hour) * 60);
	common.Second = (mjd.FracDay * 24 - common.Hour) * 3600.0 - common.Minute * 60.0;
}

/**
* @brief 简化儒略日转化为GPS时间
* @details
* 此函数将简化儒略日转化为GPS时间
* @param mjd 简化儒略日
* @param gps GPS时间
*/
void MjdTimeToGPSTime(const MJDTIME& mjd, GPSTIME& gps)
{
	gps.Week = int((mjd.Days + mjd.FracDay - 44244) / 7.0);
	gps.SecOfWeek = (mjd.Days + mjd.FracDay - 44244 - gps.Week * 7) * 86400;
}

/**
* @brief GPS时间转化为简化儒略日
* @details
* 此函数将GPS时间转化为简化儒略日
* @param gps GPS时间
* @param mjd 简化儒略日
*/
void GPSTimeToMjdTime(const GPSTIME& gps, MJDTIME& mjd)
{
	mjd.Days = 44244 + gps.Week * 7 + int(gps.SecOfWeek / 86400);
	mjd.FracDay = (gps.SecOfWeek - int(gps.SecOfWeek / 86400) * 86400) / 86400.0;
}

/**
* @brief 通用时转化为GPS时间
* @details
* 此函数将通用时转化为GPS时间
* @param common 通用时
* @param gps GPS时间
*/
void CommonTimetoGPSTime(const COMMONTIME& common, GPSTIME& gps)
{
	MJDTIME mjd;
	CommonTimetoMjdTime(common, mjd);
	MjdTimeToGPSTime(mjd, gps);
}

/**
* @brief GPS时间转化为通用时
* @details
* 此函数将GPS时间转化为通用时
* @param gps GPS时间
* @param common 通用时
*/
void GPSTimeToCommonTime(const GPSTIME& gps, COMMONTIME& common)
{
	MJDTIME mjd;
	GPSTimeToMjdTime(gps, mjd);
	MjdTimeToCommonTime(mjd, common);
}