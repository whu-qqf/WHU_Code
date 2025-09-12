#pragma once
#include<vector>
using namespace std;
class Coordinate
{
public:
	void BLH2XYZ(double*m, double*n);    //������BLHת����XYZ����
	void XYZ2BLH(double*c, double*d);    //������XYZת����BLH����
	//���ĵع�����ϵXYZ���������ϵBLH
	vector<vector<double>> XYZ2BLH(vector<vector<double>>a);
	void XYZ2NED(double*p, double*q,double*w);    //������XYZת����NED��������վ������ϵ���ĺ���
	//���ĵع�����ϵXYZ��������վ������ϵ
	vector<vector<double>> XYZ2ENU(vector<vector<double>>a, vector<vector<double>>b);
	void NED2FRD(double*x, double *t, double*a, double*g);     //������NEDת����FRD����������ϵ���ĺ���
};