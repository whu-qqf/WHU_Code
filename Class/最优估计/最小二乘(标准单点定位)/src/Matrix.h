#pragma once
#include<vector>

using namespace std;

class Matrix
{
public:
	//显示矩阵
	void display(vector<vector<int>>& a);
	//显示矩阵
	void display(vector<vector<double>>& a);
	//矩阵的乘法
	vector<vector<double>> multiplication(vector<vector<double>>& a, vector<vector<double>>& b);
	//矩阵的乘法
	vector<vector<int>> multiplication(vector<vector<int>>& a, vector<vector<int>>& b);
	//矩阵的转置
	vector<vector<double>> transposition(vector<vector<double>>& a);
	//矩阵的转置
	vector<vector<int>> transposition(vector<vector<int>>& a);
	//矩阵的行列式
	int determinant(vector<vector<int>>& a);
	//矩阵的行列式
	double determinant(vector<vector<double>>& a);
	//伴随矩阵
	vector<vector<int>> adjoint(vector<vector<int>>& a);
	//伴随矩阵
	vector<vector<double>> adjoint(vector<vector<double>>& a);
	//矩阵的逆
	vector<vector<double>> inverse(vector<vector<double>>& a);
	//矩阵的逆
	vector<vector<double>> inverse(vector<vector<int>>& a);
};