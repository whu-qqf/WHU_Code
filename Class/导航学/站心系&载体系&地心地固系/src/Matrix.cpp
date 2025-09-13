#include<iostream>
#include"Matrix.h"
void Matrix::ttmatrix(double(*a)[3], double(*b)[3], double(*c)[3])    //定义三阶矩阵与三阶矩阵的乘法
{
	int x = 0, y = 0;
	for (double *p = c[0]; p < c[0] + 9; p++)
	{
		*p = 0;
	}
	for (int i = 0; i < 9; i++)    //循环9次，求9个数
	{
		for (int j = 0; j < 3; j++)    //一个小循环完成[x][y]元素的求值
		{
			*(c[x] + y) += *(a[x] + j)*(*(b[j] + y));     //累加求每一项
		}
		y++;    //换到下一项
		if (i == 2 || i == 5 || i == 8)x++;    //换行
		if (y == 3)y = 0;    //一行求完后从下一行第一个开始求
	}
}
void Matrix::tomatrix(double(*d)[3], double *e, double*f)     //定义三阶矩阵与三维列向量的乘法
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
