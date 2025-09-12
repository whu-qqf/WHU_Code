#pragma once
#include<vector>

using namespace std;

class Matrix
{
public:
	//��ʾ����
	void display(vector<vector<int>>& a);
	//��ʾ����
	void display(vector<vector<double>>& a);
	//����ĳ˷�
	vector<vector<double>> multiplication(vector<vector<double>>& a, vector<vector<double>>& b);
	//����ĳ˷�
	vector<vector<int>> multiplication(vector<vector<int>>& a, vector<vector<int>>& b);
	//�����ת��
	vector<vector<double>> transposition(vector<vector<double>>& a);
	//�����ת��
	vector<vector<int>> transposition(vector<vector<int>>& a);
	//���������ʽ
	int determinant(vector<vector<int>>& a);
	//���������ʽ
	double determinant(vector<vector<double>>& a);
	//�������
	vector<vector<int>> adjoint(vector<vector<int>>& a);
	//�������
	vector<vector<double>> adjoint(vector<vector<double>>& a);
	//�������
	vector<vector<double>> inverse(vector<vector<double>>& a);
	//�������
	vector<vector<double>> inverse(vector<vector<int>>& a);
};