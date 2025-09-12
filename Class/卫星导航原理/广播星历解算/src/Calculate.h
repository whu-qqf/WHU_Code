#pragma once
#include <vector>
#include"ReadFile.h"

#define GM 3.986005e14 // 地球引力常数
#define OMEGA_E 7.2921151467e-5 // 地球自转角速度

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
vector<Satellite> Calculate(vector<Ephemeris> alldata);   //解算卫星位置
