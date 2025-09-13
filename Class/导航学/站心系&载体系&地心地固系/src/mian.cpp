#include<iostream>
#include<iomanip>
#include<cmath>
#include"Matrix.h"
#include"Coordinate.h"        //头文件的引入
using namespace std;

int main()
{
	Matrix Run;
	Coordinate Transfer;    //声明坐标转换对象Transfer
	double Xo[3] = { -2148744.679,4426641.029,4044655.862 }, Xt[3] = { -2148745.679,4426639.122,4045655.657 };        //站心和目标点的空间直角坐标系
	double Xl[3];        //Xl[3]用于接收坐标变换后的结果，表示目标点在站心坐标系下的坐标
	Transfer.XYZ2NED(Xo, Xt, Xl);        //调用类函数XYZ2NED，实现从空间直角坐标系到站心坐标系的转换
	cout << "目标点在站心坐标系下坐标为:（";
	for (int i = 0; i < 3; i++)
	{
		cout << setiosflags(ios::fixed)<<setprecision(10)<< Xl[i];        //输出坐标
		if (i != 2)cout << ",";
	}
	cout << ")" << endl;
	double XI[3] = { -2148746.822,4426645.265,4044653.164 }, XG[3] = { -2148746.708,4426645.350,4044653.894 };        //惯导空间直角坐标系XI，GNSS接收机的空间直角坐标系
	double Ola[3] = { 30,45,120 }, lbg[3];        //欧拉角Ola，lbg用于接收坐标转化后的结果，表示GNSS接收机在惯导载体坐标系下的坐标
	Transfer.NED2FRD(XI, XG, Ola, lbg);        //进行坐标转换
	cout << "GNSS相位中心在惯导载体坐标系下坐标为:（";
	for (int i = 0; i < 3; i++)
	{
		cout << setiosflags(ios::fixed)<<setprecision(5) << lbg[i] ;        //输出结果
		if (i != 2)cout << ",";
	}
	cout << ")" << endl;
	return 0;
}
