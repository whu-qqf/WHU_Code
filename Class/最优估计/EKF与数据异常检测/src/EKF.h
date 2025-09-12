#pragma once
#include"struct.h"
void EKF_5(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_8(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_9(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_10(double X[]/*上一个历元的结果*/, double D[]/*上一个历元的方差*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu);
