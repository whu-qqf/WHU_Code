#include<iostream>
#include<iomanip>
#include<cmath>
using namespace std;
#define a  6378137.00                                //定义常数
#define b  6356752.31424517
#define f  1 / 298.257223563 
#define e  0.0818191908426
#define e2 0.0066943799013
#define pi acos(-1)
int main()
{
	double X,Y,Z;
	cout << "请依次输入X，Y，Z：" << endl;
	cin >> X >> Y >> Z;
	void XYZ2BLH(double X,double Y,double Z);          //声明函数
	XYZ2BLH(X,Y,Z);
	return 0;
}
void BLH2XYZ(double B,double L,double H)
{
	double N = a / sqrt(1 - e2 * sin(B*pi / 180)*sin(B*pi / 180));
	double X = (N + H)*cos(B*pi / 180)*cos(L*pi / 180);
	double Y = (N + H)*cos(B*pi / 180)*sin(L*pi / 180);
	double Z = (N*(1 - e2) + H)*sin(B*pi / 180);
	cout << "经纬度分别加1°后坐标转化为XYZ分别为" << setprecision(12) << "X=" << X << ",Y=" << Y << ",Z=" << Z << endl;
}
void XYZ2BLH(double X,double Y,double Z)        
{
	double B = 0, H = 0;                           //利用X，Y，Z求B，L，H
	double L = atan(Y / X) * 180 / pi;
	if(L<0)L += 180;
	double Z0 = e *e* Z;                                    //迭代法求B
	double B0 = atan((Z + Z0) / sqrt(X*X + Y * Y));
	for (int i=0; i<=15;i++)
	{
		Z0 = e*e * sin(B0)*a / sqrt(1 - e*e * sin(B0)* sin(B0));
		B0 = atan((Z + Z0) / sqrt(X*X + Y * Y));
		B = B0;
	}
	H = sqrt(X*X + Y * Y + (Z+Z0)*(Z+Z0)) - a / sqrt(1 - e*e * sin(B0)* sin(B0));
	B = B * 180 / pi;
	cout<< "此坐标转化为BLH分别为：" << setprecision(12) << "B=" << B << ",L=" << L << ",H=" << H << endl;
	BLH2XYZ(B + 1, L + 1, H);      //加1°处理
}