#pragma once
#include"const.h"
#include"struct.h"

void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3]);    //�����źŷ���ʱ������λ�ú��ٶ�
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);    // ���㶨λ����
void SPV(EPOCHOBSDATA* Epoch, POSRES* Res);    // ������ٺ���
