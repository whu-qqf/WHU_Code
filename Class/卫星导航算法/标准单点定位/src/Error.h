#pragma once
#include"Error.h"
#include"struct.h"


double Hopfield(const double H, const double Elev);    //Hopefield�����ģ��
void DetectOutlier(EPOCHOBSDATA* Obs);    //�ֲ�������̽��

//double Klobutchar(const GPSTIME* Time, double Elve, double Azim, double RcvPos[3]);