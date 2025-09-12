#pragma once
#include "const.h"
#include"struct.h"


bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);   //���������Ӳ�
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* GPSEph, SATPVT* Mid);    //���ù㲥��������GPS����λ�á��ٶȺ��Ӳ�
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* BDSEph, SATPVT* Mid);    //���ù㲥��������BDS����λ�á��ٶȺ��Ӳ�
int EarthRotate(double UserPos[3], const GPSEPHREC* Eph, SATPVT* Mid);    //������ת����