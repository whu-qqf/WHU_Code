#include<iostream>
#include"Matrix.h"
void Matrix::ttmatrix(double(*a)[3], double(*b)[3], double(*c)[3])    //�������׾��������׾���ĳ˷�
{
	int x = 0, y = 0;
	for (double *p = c[0]; p < c[0] + 9; p++)
	{
		*p = 0;
	}
	for (int i = 0; i < 9; i++)    //ѭ��9�Σ���9����
	{
		for (int j = 0; j < 3; j++)    //һ��Сѭ�����[x][y]Ԫ�ص���ֵ
		{
			*(c[x] + y) += *(a[x] + j)*(*(b[j] + y));     //�ۼ���ÿһ��
		}
		y++;    //������һ��
		if (i == 2 || i == 5 || i == 8)x++;    //����
		if (y == 3)y = 0;    //һ����������һ�е�һ����ʼ��
	}
}
void Matrix::tomatrix(double(*d)[3], double *e, double*f)     //�������׾�������ά�������ĳ˷�
{
	for (int i = 0; i < 3; i++)
	{
		*(f + i) = 0;
	}
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			*(f + i) += *(d[i] + j)*(*(e + j));
		}
	}
}
