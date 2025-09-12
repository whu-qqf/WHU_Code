#include<cmath>
#include<vector>
#include<iostream>
#include<iomanip>
#include"Matrix.h"

using namespace std;

void Matrix::display(vector<vector<double>>& a)    //在控制台上显示矩阵
{
	for (int i = 0; i < a.size(); i++)    //行
	{
		for (int j = 0; j < a[0].size(); j++)    //列
		{
			cout <<fixed<<setprecision(6)<< a[i][j] << " ";
		}
		cout << endl;
	}
}

void Matrix::display(vector<vector<int>>& a)
{
	for (int i = 0; i < a.size(); i++)
	{
		for (int j = 0; j < a[0].size(); j++)
		{
			cout << a[i][j] << " ";
		}
		cout << endl;
	}
}

vector<vector<int>> Matrix::multiplication(vector<vector<int>>& a, vector<vector<int>>& b)    //实现矩阵相乘，返回结果
{
	vector<vector<int>> c;
	if (a[0].size() != b.size()) cout << "无法进行乘法运算\n";    //对能否进行乘法运算判断
	else
	{
		for (int i = 0; i < a.size(); i++)
		{
			vector<int>d;
			for (int j = 0; j < b[0].size(); j++)
			{
				int sum = 0;
				for (int k = 0; k < b.size(); k++)sum += a[i][k] * b[k][j];    //按照矩阵乘法法则运算
				d.push_back( sum);
			}
			c.push_back(d);
		}
	}
	return c;
}

vector<vector<double>> Matrix::multiplication(vector<vector<double>>& a, vector<vector<double>>& b)
{
	vector<vector<double>> c;
	if (a[0].size() != b.size()) cout << "无法进行乘法运算\n";
	else
	{
		for (int i = 0; i < a.size(); i++)
		{
			vector<double>d;
			for (int j = 0; j < b[0].size(); j++)
			{
				double sum = 0;
				for (int k = 0; k < b.size(); k++)sum += a[i][k] * b[k][j];
				d.push_back(sum);
			}
			c.push_back(d);
		}
	}
	return c;
}

vector<vector<int>> Matrix::transposition(vector<vector<int>>& a)  //实现矩阵转置，返回转置矩阵
{
	vector<vector<int>> c;
	for (int i = 0; i < a[0].size(); i++)    //行
	{
		vector<int>d;
		for (int j = 0; j < a.size(); j++)    //列
		{
			int num ;
			num = a[j][i];
			d.push_back(num);
		}
		c.push_back(d);
	}
	return c;
}

vector<vector<double>> Matrix::transposition(vector<vector<double>>& a)
{
	vector<vector<double>> c;
	for (int i = 0; i < a[0].size(); i++)
	{
		vector<double>d;
		for (int j = 0; j < a.size(); j++)
		{
			double num;
			num = a[j][i];
			d.push_back(num);
		}
		c.push_back(d);
	}
	return c;
}

int Matrix::determinant(vector<vector<int>>& a)    //计算矩阵行列式，返回结果
{
	int c;
	int n = a.size();
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求矩阵的行列式\n";    //判断是否为方阵
	/*阶数为1和2的运算*/
	if (n == 1)c = a[0][0];
	else if (n == 2)c = a[0][0] * a[1][1] - a[0][1] * a[1][0];
	/*高阶运算*/
	else 
	{
		int sum = 0;
		for (int i = 0; i < n; i++) 
		{
		    vector<vector<int>> submatrix(n - 1, vector<int>(n - 1));    //定义子矩阵
			for (int j = 1; j < n; j++) 
			{
			    int m = 0;
				for (int k = 0; k < n; ++k) 
				{
					if (k == i) continue;
					submatrix[j - 1][m] = a[j][k];
					m++;
				}
			}
			sum += a[0][i] * determinant(submatrix) * (i % 2 == 0 ? 1 : -1);    //递归求行列式
		}
		c = sum;
	}
	return c;
}

double Matrix::determinant(vector<vector<double>>& a)
{
	double c;
	int n = a.size();
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求矩阵的行列式\n";
	if (n == 1)c = a[0][0];
	else if (n == 2)c = a[0][0] * a[1][1] - a[0][1] * a[1][0];
	else
	{
		double sum = 0;
		for (int i = 0; i < n; i++)
		{
			vector<vector<double>> submatrix(n - 1, vector<double>(n - 1));
			for (int j = 1; j < n; j++)
			{
				int m = 0;
				for (int k = 0; k < n; ++k)
				{
					if (k == i) continue;
					submatrix[j - 1][m] = a[j][k];
					m++;
				}
			}
			sum += a[0][i] * determinant(submatrix) * (i % 2 == 0 ? 1 : -1);
		}
		c = sum;
	}
	return c;
}

vector<vector<int>> Matrix::adjoint(vector<vector<int>>& a)    //求矩阵的伴随矩阵
{
	int n = a.size();
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求伴随矩阵\n";    //判断是否为方阵
	vector<vector<int>> c(n, vector<int>(n));    //结果矩阵
	int m = 0, o = 0;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{

			vector<vector<int>> subMatrix(n - 1, vector<int>(n - 1));    //定义子矩阵
			for (int k = 0; k < n; k++)
			{

				if (k == i) continue;    //当前行就跳出
				for (int l = 0; l < n; l++)
				{
					if (l == j) continue;    //当前列就跳出
					subMatrix[m][o] = a[k][l];
					o++;
				}
				o = 0;
				m++;
			}
			m = 0;
			c[i][j] = determinant(subMatrix) * ((i + j) % 2 == 0 ? 1 : -1);    //求代数余子式
		}
	}
	return transposition(c);
}

vector<vector<double>> Matrix::adjoint(vector<vector<double>>& a)
{
	int n = a.size();
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求伴随矩阵\n";
	vector<vector<double>> c(n, vector<double>(n));
	int m = 0, o = 0;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{

			vector<vector<double>> subMatrix(n - 1, vector<double>(n - 1));
			for (int k = 0; k < n; k++)
			{

				if (k == i) continue;
				for (int l = 0; l < n; l++)
				{
					if (l == j) continue;
					subMatrix[m][o] = a[k][l];
					o++;
				}
				o = 0;
				m++;
			}
			m = 0;
			c[i][j] = 1.0*determinant(subMatrix) * ((i + j) % 2 == 0 ? 1 : -1);
		}
	}
	return transposition(c);
}

vector<vector<double>> Matrix::inverse(vector<vector<int>>& a)    //求矩阵的转置矩阵
{
	int n = a.size();
	vector<vector<double>> c(n, vector<double>(n));
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求逆\n";
	else
	{
		if (determinant(a) == 0) cout << "行列式为0，无法求逆\n";
		else
		{
			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					c[i][j] = adjoint(a)[i][j]*1.0 / determinant(a);    //调用前面的矩阵求逆和伴随矩阵
				}
			}
		}
	}
	return c;
}

vector<vector<double>> Matrix::inverse(vector<vector<double>>& a)
{
	int n = a.size();
	vector<vector<double>> c(n, vector<double>(n));
	if (a.size() != a[0].size()) cout << "行数与列数不等，无法求逆\n";
	else
	{
		if (determinant(a) == 0) cout << "行列式为0，无法求逆\n";
		else
		{
			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					c[i][j] = adjoint(a)[i][j] * 1.0 / determinant(a);
				}
			}
		}
	}
	return c;
}