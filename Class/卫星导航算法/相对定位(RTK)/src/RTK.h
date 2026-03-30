#pragma once
#include<iostream>
#include"const.h"
#include"struct.h"
#include"sockets.h"

int TimeSyn(FILE* FObs_base, FILE* FObs_rover, RAWDATA* rawdata);
int RealTimeSyn(SOCKET* NetGps1, SOCKET* NetGps2, RAWDATA* rawdata);

void FormSDEpochObs(const EPOCHOBSDATA* EpkB, const EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs);
void DetectCirclSlip(SDEPOCHOBS* SDObs);
void RefSatSelection(const EPOCHOBSDATA* EpkB, const EPOCHOBSDATA* EpkR, SDEPOCHOBS* SDObs, DDCOBS* DDObs);

int RTK(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* SPPPos_rover, POSRES* RTKPos_Rover);
//int EKF_RTK(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* SPPPos_rover, XYZ* RTKPos_Rover, double Ambi[],double PreX[],double PreP[]);
void EKFINIT(POSRES* SPPPos_rover, EKFRTK* EkfRTK);
void TIMEPRED(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover,EKFRTK* EkfRTK);
void EKFUPDATE(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK);

void RTK_FIXED(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, double Fixed[]);
void KF_FIXED1(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK, double Fixed[]);
void KF_FIXED2(RAWDATA* rawdata, SDEPOCHOBS* SDObs, DDCOBS* DDObs, POSRES* RTKPos_Rover, EKFRTK* EkfRTK, double Fixed[]);
