#pragma once
#include"struct.h"
void EKF_5(double X[]/*��һ����Ԫ�Ľ��*/, double D[]/*��һ����Ԫ�ķ���*/, EKFOBS* obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_8(double X[]/*��һ����Ԫ�Ľ��*/, double D[]/*��һ����Ԫ�ķ���*/, EKFOBS* obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_9(double X[]/*��һ����Ԫ�Ľ��*/, double D[]/*��һ����Ԫ�ķ���*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu);
void EKF_10(double X[]/*��һ����Ԫ�Ľ��*/, double D[]/*��һ����Ԫ�ķ���*/, EKFOBS* Obs, ReferencePos* refPos, ENUCOOR& enu);
