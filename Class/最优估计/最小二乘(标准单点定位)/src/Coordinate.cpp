#include<iostream>
#include<cmath>
#include"Coordinate.h"
#include"Matrix.h"         //引入头文件
#include<vector>
#define a  6378137.00                               
#define b  6356752.31424517
#define f  1 / 298.257223563 
#define e  0.0818191908426
#define e2 e*e
#define pi acos(-1)                //定义常量
using namespace std;
Matrix nmat;                //创建矩阵运算类型对象Run
void Coordinate::BLH2XYZ(double*m, double*n)    //定义BLH到XYZ坐标转换函数，m是大地坐标，n是转换后的空间直角坐标
{
	double N = a / sqrt(1 - e2 * sin((*m)*pi / 180)*sin((*m)*pi / 180));
	*n = (N + *(m + 2))*cos((*m)*pi / 180)*cos(*(m + 1)*pi / 180);
	*(n + 1) = (N + *(m + 2))*cos((*m)*pi / 180)*sin(*(m + 1)*pi / 180);
	*(n + 2) = (N*(1 - e2) + *(m + 2))*sin((*m)*pi / 180);
}
void Coordinate::XYZ2BLH(double*c, double*d)    //定义XYZ到BLH坐标转换函数，c的空间直角坐标，d是转换后的大地坐标
{
	double B = 0, H = 0;                           
	double L = atan(*(c + 1) / (*c));
	if (L < 0)L += pi;
	double Z0 = e * e* (*(c + 2));      //迭代法求B
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
	*(d + 2) = H;                          //将最后的结果记录在数组中
}
void Coordinate::XYZ2NED(double*Xo, double*Xt,double*Xl)    //定义XYZ到NED坐标转换函数，Xo，Xt分别是站心和目标点的空间直角坐标，Xl表示目标点的站心坐标
{
	double B0[3], m[3];    //B0是Xo站心的大地坐标系，m是一个中间量
	XYZ2BLH(Xo, B0);       //将站心的空间直角坐标转换成大地坐标
	double B = B0[0] , L = B0[1] ;
	double Cel[3][3] = { {-sin(B)*cos(L),-sin(B)*sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B)*cos(L),-cos(B)*sin(L),-sin(B)} };    //坐标变换矩阵
	for (int i = 0; i < 3; i++)
	{
		m[i] = *(Xt+i) - *(Xo+i);    //目标点和站心的坐标向量
	}
	//Run.tomatrix(Cel, m, Xl);         //Xl=Cel*m,进行坐标矩阵的变换
}
void Coordinate::NED2FRD(double*x, double *t, double*A, double*g)    //定义NED到FRD坐标转换函数，x和t分别为载体和目标点的空间直角坐标，A表示载体的欧拉角，g表示目标点的载体坐标
{
	double R = (*A)*pi / 180, P = *(A + 1)*pi / 180, Y = *(A + 2)*pi / 180;
	double Bx[3],deta[3],Ceb[3][3];         //Bx表示载体的大地坐标，deta是中间量，接收目标点和载体的坐标向量，Ceb是坐标变换矩阵相乘的结果
	XYZ2BLH(x, Bx);               //将载体的空间直角坐标转换成大地坐标
	double B = Bx[0] , L = Bx[1] ;
	double Cel[3][3] = { {-sin(B)*cos(L),-sin(B)*sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B)*cos(L),-cos(B)*sin(L),-sin(B)} };     //坐标变换矩阵
	double Clb[3][3] = { {cos(P)*cos(Y),cos(P)*sin(Y),-sin(P)},{sin(R)*sin(P)*cos(Y) - cos(R)*sin(Y),sin(R)*sin(P)*sin(Y) + cos(R)*cos(Y),sin(R)*cos(P)},{cos(R)*sin(P)*cos(Y) + sin(R)*sin(Y),cos(R)*sin(P)*sin(Y) - sin(R)*cos(Y),cos(R)*cos(P)} };
	for (int i = 0; i < 3; i++)
	{
		deta[i] = *(t + i) - *(x + i);     //目标点和载体的坐标向量
	}
	//Run.ttmatrix(Clb, Cel, Ceb);    //Ceb=Clb*Cel
	//Run.tomatrix(Ceb, deta, g);     //g=Ceb*deta,进行坐标变换
}
vector<vector<double>> Coordinate::XYZ2ENU(vector<vector<double>>m, vector<vector<double>>n)
{
	Matrix mat;
	vector<double>h;
	vector<vector<double>> BO, s,r;
	vector<vector<double>>p = { {0},{0},{0} };
	BO = XYZ2BLH(m);
	double B = BO[0][0], L = BO[1][0];
	vector<vector<double>> Cel = { {-sin(B) * cos(L),-sin(B) * sin(L),cos(B)},{-sin(L),cos(L),0},{-cos(B) * cos(L),-cos(B) * sin(L),-sin(B)} };    //坐标变换矩阵
	for (int i = 0; i < 3; i++)
	{
		p[i][0] = n[i][0] - m[i][0];    //目标点和站心的坐标向量
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
	double Z0 = e * e * (m[2][0]);      //迭代法求B
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
