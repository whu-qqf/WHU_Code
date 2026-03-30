#pragma once
#include"struct.h"

void MjdTimeToCommonTime(const MJDTIME& mjd, COMMONTIME& common);   //将简化儒略日转换为通用时
void MjdTimeToGPSTime(const MJDTIME& mjd, GPSTIME& gps);   //将简化儒略日转换为GPS时间
void GPSTimeToMjdTime(const GPSTIME& gps, MJDTIME& mjd);   //将GPS时间转换为简化儒略日
void GPSTimeToCommonTime(const GPSTIME& gps, COMMONTIME& common);   //将GPS时间转换为通用时
void CommonTimetoGPSTime(const COMMONTIME& common, GPSTIME& gps);   //将通用时转换为GPS时间
void CommonTimetoMjdTime(const COMMONTIME& common, MJDTIME& mjd);   //将通用时转换为简化儒略日