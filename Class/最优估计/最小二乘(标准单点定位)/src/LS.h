#pragma once
#include<vector>
#include<string>
using namespace std;
class LS
{
public:
	//进行最小二乘解算
	vector<vector<vector<double>>>leastsqure(double*m,vector<vector<double>>& data);
};