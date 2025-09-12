#pragma once
#include"Error.h"
#include"struct.h"


double Hopfield(const double H, const double Elev);    //Hopefield电离层模型
void DetectOutlier(EPOCHOBSDATA* Obs);    //粗差与周跳探测

//double Klobutchar(const GPSTIME* Time, double Elve, double Azim, double RcvPos[3]);