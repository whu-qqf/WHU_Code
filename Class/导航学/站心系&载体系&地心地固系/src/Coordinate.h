#pragma once
class Coordinate
{
public:
	void BLH2XYZ(double*m, double*n);    //������BLHת����XYZ����
	void XYZ2BLH(double*c, double*d);    //������XYZת����BLH����
	void XYZ2NED(double*p, double*q,double*w);    //������XYZת����NED��վ������ϵ���ĺ���
	void NED2FRD(double*x, double *t, double*a, double*g);     //������NEDת����FRD����������ϵ���ĺ���
};