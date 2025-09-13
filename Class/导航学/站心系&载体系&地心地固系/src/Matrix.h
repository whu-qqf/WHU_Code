#pragma once
class Matrix
{
public:
	void ttmatrix(double(*a)[3], double(*b)[3], double(*c)[3]);    //声明三阶矩阵乘三阶矩阵计算函数
	void tomatrix(double(*d)[3], double *e, double*f);    //声明三阶矩阵成三维列向量计算函数
};