#pragma once
#include"const.h"
#include"struct.h"

void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3]);    //计算信号发射时刻卫星位置和速度
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);    // 单点定位函数
void SPV(EPOCHOBSDATA* Epoch, POSRES* Res);    // 单点测速函数
