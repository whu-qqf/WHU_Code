#include<iostream>
#include<iomanip>
#include<cmath>
#include"Matrix.h"
#include"Coordinate.h"        //ͷ�ļ�������
using namespace std;

int main()
{
	Matrix Run;
	Coordinate Transfer;    //��������ת������Transfer
	double Xo[3] = { -2148744.679,4426641.029,4044655.862 }, Xt[3] = { -2148745.679,4426639.122,4045655.657 };        //վ�ĺ�Ŀ���Ŀռ�ֱ������ϵ
	double Xl[3];        //Xl[3]���ڽ�������任��Ľ������ʾĿ�����վ������ϵ�µ�����
	Transfer.XYZ2NED(Xo, Xt, Xl);        //�����ຯ��XYZ2NED��ʵ�ִӿռ�ֱ������ϵ��վ������ϵ��ת��
	cout << "Ŀ�����վ������ϵ������Ϊ:��";
	for (int i = 0; i < 3; i++)
	{
		cout << setiosflags(ios::fixed)<<setprecision(10)<< Xl[i];        //�������
		if (i != 2)cout << ",";
	}
	cout << ")" << endl;
	double XI[3] = { -2148746.822,4426645.265,4044653.164 }, XG[3] = { -2148746.708,4426645.350,4044653.894 };        //�ߵ��ռ�ֱ������ϵXI��GNSS���ջ��Ŀռ�ֱ������ϵ
	double Ola[3] = { 30,45,120 }, lbg[3];        //ŷ����Ola��lbg���ڽ�������ת����Ľ������ʾGNSS���ջ��ڹߵ���������ϵ�µ�����
	Transfer.NED2FRD(XI, XG, Ola, lbg);        //��������ת��
	cout << "GNSS��λ�����ڹߵ���������ϵ������Ϊ:��";
	for (int i = 0; i < 3; i++)
	{
		cout << setiosflags(ios::fixed)<<setprecision(5) << lbg[i] ;        //������
		if (i != 2)cout << ",";
	}
	cout << ")" << endl;
	return 0;
}
