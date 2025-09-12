#include<iostream>
#include<cmath>
#include"Coordinate.h"
#include"Matrix.h"         //����ͷ�ļ�
#include<vector>
#define a  6378137.00                               
#define b  6356752.31424517
#define f  1 / 298.257223563 
#define e  0.0818191908426
#define e2 e*e
#define pi acos(-1)                //���峣��
using namespace std;
Matrix nmat;                //���������������Ͷ���Run
void Coordinate::BLH2XYZ(double*m, double*n)    //����BLH��XYZ����ת��������m�Ǵ�����꣬n��ת����Ŀռ�ֱ������
{
	double N = a / sqrt(1 - e2 * sin((*m)*pi / 180)*sin((*m)*pi / 180));
	*n = (N + *(m + 2))*cos((*m)*pi / 180)*cos(*(m + 1)*pi / 180);
	*(n + 1) = (N + *(m + 2))*cos((*m)*pi / 180)*sin(*(m + 1)*pi / 180);
	*(n + 2) = (N*(1 - e2) + *(m + 2))*sin((*m)*pi / 180);
}
void Coordinate::XYZ2BLH(double*c, double*d)    //����XYZ��BLH����ת��������c�Ŀռ�ֱ�����꣬d��ת����Ĵ������
{
	double B = 0, H = 0;                           
	double L = atan(*(c + 1) / (*c));
	if (L < 0)L += pi;
	double Z0 = e * e* (*(c + 2));      //��������B
	double B0 = atan((*(c + 2) + Z0) / sqrt((*c)*(*c) + *(c + 1)* (*(c + 1))));
	do
	{
		Z0 = e * e * sin(B0)*a / sqrt(1 - e * e * sin(B0)* sin(B0));
		B0 = atan((*(c + 2) + Z0) / sqrt(*c*(*c) + *(c + 1)* (*(c + 1))));
		if (abs(B0 - B) < 1e-20)
		{
			B = B0; break;
		}
		else B = B0;
	}while(true) ;
	H = sqrt(*c*(*c) + *(c + 1)* (*(c + 1)) + ((*(c + 2)) + Z0)*(*(c + 2) + Z0)) - a / sqrt(1 - e * e * sin(B0)* sin(B0));
	*d = B;                               
	*(d + 1) = L;
	*(d + 2) = H;                          //�����Ľ����¼��������
}
void Coordinate::XYZ2NED(double*Xo, double*Xt,double*Xl)    //����XYZ��NED����ת��������Xo��Xt�ֱ���վ�ĺ�Ŀ���Ŀռ�ֱ�����꣬Xl��ʾĿ����վ������
{
	double B0[3], m[3];    //B0��Xoվ�ĵĴ������ϵ��m��һ���м���
	XYZ2BLH(Xo, B0);       //��վ�ĵĿռ�ֱ������ת���ɴ������
	double B = B0[0] , L = B0[1] ;
	double Cel[3][3] = { {-sin(B)*cos(L),-sin(B)*sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B)*cos(L),-cos(B)*sin(L),-sin(B)} };    //����任����
	for (int i = 0; i < 3; i++)
	{
		m[i] = *(Xt+i) - *(Xo+i);    //Ŀ����վ�ĵ���������
	}
	//Run.tomatrix(Cel, m, Xl);         //Xl=Cel*m,�����������ı任
}
void Coordinate::NED2FRD(double*x, double *t, double*A, double*g)    //����NED��FRD����ת��������x��t�ֱ�Ϊ�����Ŀ���Ŀռ�ֱ�����꣬A��ʾ�����ŷ���ǣ�g��ʾĿ������������
{
	double R = (*A)*pi / 180, P = *(A + 1)*pi / 180, Y = *(A + 2)*pi / 180;
	double Bx[3],deta[3],Ceb[3][3];         //Bx��ʾ����Ĵ�����꣬deta���м���������Ŀ�������������������Ceb������任������˵Ľ��
	XYZ2BLH(x, Bx);               //������Ŀռ�ֱ������ת���ɴ������
	double B = Bx[0] , L = Bx[1] ;
	double Cel[3][3] = { {-sin(B)*cos(L),-sin(B)*sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B)*cos(L),-cos(B)*sin(L),-sin(B)} };     //����任����
	double Clb[3][3] = { {cos(P)*cos(Y),cos(P)*sin(Y),-sin(P)},{sin(R)*sin(P)*cos(Y) - cos(R)*sin(Y),sin(R)*sin(P)*sin(Y) + cos(R)*cos(Y),sin(R)*cos(P)},{cos(R)*sin(P)*cos(Y) + sin(R)*sin(Y),cos(R)*sin(P)*sin(Y) - sin(R)*cos(Y),cos(R)*cos(P)} };
	for (int i = 0; i < 3; i++)
	{
		deta[i] = *(t + i) - *(x + i);     //Ŀ�����������������
	}
	//Run.ttmatrix(Clb, Cel, Ceb);    //Ceb=Clb*Cel
	//Run.tomatrix(Ceb, deta, g);     //g=Ceb*deta,��������任
}
vector<vector<double>> Coordinate::XYZ2ENU(vector<vector<double>>m, vector<vector<double>>n)
{
	Matrix mat;
	vector<double>h;
	vector<vector<double>> BO, s,r;
	vector<vector<double>>p = { {0},{0},{0} };
	BO = XYZ2BLH(m);
	double B = BO[0][0], L = BO[1][0];
	vector<vector<double>> Cel = { {-sin(B) * cos(L),-sin(B) * sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B) * cos(L),-cos(B) * sin(L),-sin(B)} };    //����任����
	for (int i = 0; i < 3; i++)
	{
		p[i][0] = n[i][0] - m[i][0];    //Ŀ����վ�ĵ���������
	}
	s=mat.multiplication(Cel, p);
	h.push_back(s[1][0]);
	r.push_back(h);
	h.clear();
	h.push_back(s[0][0]);
	r.push_back(h);
	h.clear();
	h.push_back(-s[2][0]);
	r.push_back(h);
	return r;
}
vector<vector<double>> Coordinate::XYZ2BLH(vector<vector<double>>m)
{
	vector<vector<double>>n;
	vector<double>matr;
	double B = 0, H = 0;
	double L = atan(m[1][0] / m[0][0]);
	if (L < 0)L += pi;
	double Z0 = e * e * (m[2][0]);      //��������B
	double B0 = atan((m[2][0] + Z0) / sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]));
	do
	{
		Z0 = e * e * sin(B0) * a / sqrt(1 - e * e * sin(B0) * sin(B0));
		B0 = atan((m[2][0] + Z0) / sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]));
		if (abs(B0 - B) < 1e-20)
		{
			B = B0; break;
		}
		else B = B0;
	} while (true);
	H = sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0] + (m[2][0] + Z0) * (m[2][0] + Z0)) - a / sqrt(1 - e * e * sin(B0) * sin(B0));
	matr.push_back(B);
	n.push_back(matr);
	matr.clear();
	matr.push_back(L);
	n.push_back(matr);
	matr.clear();
	matr.push_back(H);
	n.push_back(matr);
	return n;
}
