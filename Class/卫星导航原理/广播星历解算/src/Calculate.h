#pragma once
#include <vector>
#include"ReadFile.h"

#define GM 3.986005e14 // ������������
#define OMEGA_E 7.2921151467e-5 // ������ת���ٶ�

#define BDS_GM 3.986004418e14
#define BDS_OMEGA_E 7.2921150e-5
using namespace std;
struct Satellite
{
	string satellitename;
	double x;
	double y;
	double z;
	int time;
};
vector<Satellite> Calculate(vector<Ephemeris> alldata);   //��������λ��
