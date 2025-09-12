#include<vector>
#include<cmath>
#include<iostream>
#include<iomanip>
#include"LS.h"
#include"Matrix.h"

using namespace std;

vector<vector<vector<double>>> LS::leastsqure(double*m,vector<vector<double>>& data)
{

	Matrix mat;    //定义矩阵对象
	vector<vector<double>>result = { {0}, {0}, {0},{0} };    //定义接收机坐标结果向量
	vector<vector<double>>lsresult;    //定义最小二乘解结果向量
	double sigma2, sigma;    //定义验后单位权方差和中误差
	int n = data.size();    //传入观测值的个数
	vector<vector<double>> H;    //定义雅可比矩阵H
	vector<vector<double>> Ht;    //定义H的转置矩阵Ht
	vector<vector<double>> Z;    //定义观测值向量Z
	vector<vector<double>> D(n,vector<double>(n));    //定义误差模型方差矩阵D
	vector<vector<double>> W(n,vector<double>(n));    //定义D的逆矩阵W
	vector<double>h;    //定义中间一维向量
	vector<double> s(n);    //定义距离s向量
	vector<vector<double>>HtW;    //定义Ht*W矩阵
	vector<vector<double>>Cofactor;    //定义协因数矩阵Cofactor=Ht*W*H
	vector<vector<double>>inCofactor;    //定义协因数矩阵的逆矩阵
	vector<vector<double>>hx_z;    //定义H*X-Z矩阵
	vector<vector<double>>hx;    //定义H*X矩阵
	vector<vector<double>>hx_zt;    //定义H*X-Z的逆矩阵
	vector<vector<double>>matrix;    //定义中间二维矩阵
	vector<vector<double>>varx;    //定义验后估计参数方程矩阵
	double delta;     //定义迭代前后两次结果的差值
	vector<vector<vector<double>>>allresult;    //定义最小二乘结果三维矩阵，存储顺序为：解算坐标->验后单位权中误差->验后估计参数方差
	do
	{
		/*清空前一次迭代的H矩阵和Z向量*/
		H.clear();
		Z.clear();
		/*最小二乘迭代解算*/
		for (int i = 0; i < n; i++)
		{
			s[i] = sqrt((*(m)-data[i][0]) * (*(m)-data[i][0]) + (*(m + 1) - data[i][1]) * (*(m + 1) - data[i][1]) + (*(m + 2) - data[i][2]) * (*(m + 2) - data[i][2]));    //距离S
			h.push_back((*(m)-data[i][0]) / s[i]); h.push_back((*(m + 1) - data[i][1]) / s[i]); h.push_back((*(m + 2) - data[i][2]) / s[i]); h.push_back(1.0);    //雅可比矩阵H的一行
			H.push_back(h);
			h.clear();
			h.push_back(data[i][3] - s[i] - *(m + 3));    //观测值Z的一行
			Z.push_back(h);
			h.clear();
			D[i][i] = data[i][4];    //协方差矩阵赋值
			W[i][i] = 1.0 / data[i][4];    //协方差阵的逆矩阵赋值
		}
		Ht = mat.transposition(H);    //转置运算
		HtW = mat.multiplication(Ht, W);    //相乘运算
		Cofactor = mat.multiplication(HtW, H);
		inCofactor = mat.inverse(Cofactor);    //求逆运算
		matrix = mat.multiplication(inCofactor, HtW);
		lsresult = mat.multiplication(matrix, Z);
		for (int i = 0; i < 4; i++)
		{
			result[i][0] =lsresult[i][0] + *(m + i);    //获得接收机坐标位置
		}
		delta = sqrt((*(m)-result[0][0]) * (*(m)-result[0][0]) + (*(m + 1) - result[1][0]) * (*(m + 1) - result[1][0]) + (*(m + 2) - result[2][0]) * (*(m + 2) - result[2][0]) + (*(m + 3) - result[3][0]) * (*(m + 3) - result[3][0]));    //迭代前后差值
		for (int i = 0; i < 4; i++)
		{
			*(m + i) = result[i][0];    //初值更新为上一次迭代结果
		}
		
	} while (delta > 1e-4);    //阈值循环
	std::cout <<fixed<<setprecision(4)<< "x=" << result[0][0] << "\ny=" << result[1][0] << "\nz=" << result[2][0] << "\nt=" << result[3][0];
	/*计算验后单位权方差和中误差*/
	hx = mat.multiplication(H, lsresult);
	for (int i = 0; i < n; i++)
	{
		h.push_back(hx[i][0]-Z[i][0]);
		hx_z.push_back(h);
		h.clear();
	}
	hx_zt = mat.transposition(hx_z);
	matrix = mat.multiplication(hx_zt, W);
	sigma2 = mat.multiplication(matrix, hx_z)[0][0]/(n-4);
	sigma = sqrt(sigma2);
	/*计算验后估计参数方差矩阵*/
	for (int i = 0; i < inCofactor.size(); i++)
	{
		for (int j = 0; j < inCofactor.size(); j++)
		{
			h.push_back(sigma2 * inCofactor[i][j]);
		}
		varx.push_back(h);
		h.clear();
	}
	std::cout << endl << sigma<<endl;
	mat.display(varx);    //展示矩阵
	/*存入结果并返回*/
	allresult.push_back(result);
	vector<vector<double>> t;
	h.push_back(sigma);
	t.push_back(h);
	allresult.push_back(t);
	allresult.push_back(varx);
	return allresult;
}
