#pragma once
#include "const.h"
#include"struct.h"


bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);   //计算卫星钟差
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* GPSEph, SATPVT* Mid);    //利用广播星历计算GPS卫星位置、速度和钟差
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* BDSEph, SATPVT* Mid);    //利用广播星历计算BDS卫星位置、速度和钟差
int EarthRotate(double UserPos[3], const GPSEPHREC* Eph, SATPVT* Mid);    //地球自转改正