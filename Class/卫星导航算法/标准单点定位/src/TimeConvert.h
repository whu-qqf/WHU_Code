#pragma once
#include"struct.h"

void MjdTimeToCommonTime(const MJDTIME& mjd, COMMONTIME& common);   //����������ת��Ϊͨ��ʱ
void MjdTimeToGPSTime(const MJDTIME& mjd, GPSTIME& gps);   //����������ת��ΪGPSʱ��
void GPSTimeToMjdTime(const GPSTIME& gps, MJDTIME& mjd);   //��GPSʱ��ת��Ϊ��������
void GPSTimeToCommonTime(const GPSTIME& gps, COMMONTIME& common);   //��GPSʱ��ת��Ϊͨ��ʱ
void CommonTimetoGPSTime(const COMMONTIME& common, GPSTIME& gps);   //��ͨ��ʱת��ΪGPSʱ��
void CommonTimetoMjdTime(const COMMONTIME& common, MJDTIME& mjd);   //��ͨ��ʱת��Ϊ��������