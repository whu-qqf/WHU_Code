#include<cmath>
#include<vector>
#include<iostream>
#include<iomanip>
#include"Matrix.h"

using namespace std;

void Matrix::display(vector<vector<double>>& a)    //�ڿ���̨����ʾ����
{
	for (int i = 0; i < a.size(); i++)    //��
	{
		for (int j = 0; j < a[0].size(); j++)    //��
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

vector<vector<int>> Matrix::multiplication(vector<vector<int>>& a, vector<vector<int>>& b)    //ʵ�־�����ˣ����ؽ��
{
	vector<vector<int>> c;
	if (a[0].size() != b.size()) cout << "�޷����г˷�����\n";    //���ܷ���г˷������ж�
	else
	{
		for (int i = 0; i < a.size(); i++)
		{
			vector<int>d;
			for (int j = 0; j < b[0].size(); j++)
			{
				int sum = 0;
				for (int k = 0; k < b.size(); k++)sum += a[i][k] * b[k][j];    //���վ���˷���������
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
	if (a[0].size() != b.size()) cout << "�޷����г˷�����\n";
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

vector<vector<int>> Matrix::transposition(vector<vector<int>>& a)  //ʵ�־���ת�ã�����ת�þ���
{
	vector<vector<int>> c;
	for (int i = 0; i < a[0].size(); i++)    //��
	{
		vector<int>d;
		for (int j = 0; j < a.size(); j++)    //��
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

int Matrix::determinant(vector<vector<int>>& a)    //�����������ʽ�����ؽ��
{
	int c;
	int n = a.size();
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷�����������ʽ\n";    //�ж��Ƿ�Ϊ����
	/*����Ϊ1��2������*/
	if (n == 1)c = a[0][0];
	else if (n == 2)c = a[0][0] * a[1][1] - a[0][1] * a[1][0];
	/*�߽�����*/
	else 
	{
		int sum = 0;
		for (int i = 0; i < n; i++) 
		{
		    vector<vector<int>> submatrix(n - 1, vector<int>(n - 1));    //�����Ӿ���
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
			sum += a[0][i] * determinant(submatrix) * (i % 2 == 0 ? 1 : -1);    //�ݹ�������ʽ
		}
		c = sum;
	}
	return c;
}

double Matrix::determinant(vector<vector<double>>& a)
{
	double c;
	int n = a.size();
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷�����������ʽ\n";
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

vector<vector<int>> Matrix::adjoint(vector<vector<int>>& a)    //�����İ������
{
	int n = a.size();
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷���������\n";    //�ж��Ƿ�Ϊ����
	vector<vector<int>> c(n, vector<int>(n));    //�������
	int m = 0, o = 0;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{

			vector<vector<int>> subMatrix(n - 1, vector<int>(n - 1));    //�����Ӿ���
			for (int k = 0; k < n; k++)
			{

				if (k == i) continue;    //��ǰ�о�����
				for (int l = 0; l < n; l++)
				{
					if (l == j) continue;    //��ǰ�о�����
					subMatrix[m][o] = a[k][l];
					o++;
				}
				o = 0;
				m++;
			}
			m = 0;
			c[i][j] = determinant(subMatrix) * ((i + j) % 2 == 0 ? 1 : -1);    //���������ʽ
		}
	}
	return transposition(c);
}

vector<vector<double>> Matrix::adjoint(vector<vector<double>>& a)
{
	int n = a.size();
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷���������\n";
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

vector<vector<double>> Matrix::inverse(vector<vector<int>>& a)    //������ת�þ���
{
	int n = a.size();
	vector<vector<double>> c(n, vector<double>(n));
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷�����\n";
	else
	{
		if (determinant(a) == 0) cout << "����ʽΪ0���޷�����\n";
		else
		{
			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					c[i][j] = adjoint(a)[i][j]*1.0 / determinant(a);    //����ǰ��ľ�������Ͱ������
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
	if (a.size() != a[0].size()) cout << "�������������ȣ��޷�����\n";
	else
	{
		if (determinant(a) == 0) cout << "����ʽΪ0���޷�����\n";
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