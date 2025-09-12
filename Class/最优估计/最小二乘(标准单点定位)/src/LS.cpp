#include<vector>
#include<cmath>
#include<iostream>
#include<iomanip>
#include"LS.h"
#include"Matrix.h"

using namespace std;

vector<vector<vector<double>>> LS::leastsqure(double*m,vector<vector<double>>& data)
{

	Matrix mat;    //����������
	vector<vector<double>>result = { {0}, {0}, {0},{0} };    //������ջ�����������
	vector<vector<double>>lsresult;    //������С���˽�������
	double sigma2, sigma;    //�������λȨ����������
	int n = data.size();    //����۲�ֵ�ĸ���
	vector<vector<double>> H;    //�����ſɱȾ���H
	vector<vector<double>> Ht;    //����H��ת�þ���Ht
	vector<vector<double>> Z;    //����۲�ֵ����Z
	vector<vector<double>> D(n,vector<double>(n));    //�������ģ�ͷ������D
	vector<vector<double>> W(n,vector<double>(n));    //����D�������W
	vector<double>h;    //�����м�һά����
	vector<double> s(n);    //�������s����
	vector<vector<double>>HtW;    //����Ht*W����
	vector<vector<double>>Cofactor;    //����Э��������Cofactor=Ht*W*H
	vector<vector<double>>inCofactor;    //����Э��������������
	vector<vector<double>>hx_z;    //����H*X-Z����
	vector<vector<double>>hx;    //����H*X����
	vector<vector<double>>hx_zt;    //����H*X-Z�������
	vector<vector<double>>matrix;    //�����м��ά����
	vector<vector<double>>varx;    //���������Ʋ������̾���
	double delta;     //�������ǰ�����ν���Ĳ�ֵ
	vector<vector<vector<double>>>allresult;    //������С���˽����ά���󣬴洢˳��Ϊ����������->���λȨ�����->�����Ʋ�������
	do
	{
		/*���ǰһ�ε�����H�����Z����*/
		H.clear();
		Z.clear();
		/*��С���˵�������*/
		for (int i = 0; i < n; i++)
		{
			s[i] = sqrt((*(m)-data[i][0]) * (*(m)-data[i][0]) + (*(m + 1) - data[i][1]) * (*(m + 1) - data[i][1]) + (*(m + 2) - data[i][2]) * (*(m + 2) - data[i][2]));    //����S
			h.push_back((*(m)-data[i][0]) / s[i]); h.push_back((*(m + 1) - data[i][1]) / s[i]); h.push_back((*(m + 2) - data[i][2]) / s[i]); h.push_back(1.0);    //�ſɱȾ���H��һ��
			H.push_back(h);
			h.clear();
			h.push_back(data[i][3] - s[i] - *(m + 3));    //�۲�ֵZ��һ��
			Z.push_back(h);
			h.clear();
			D[i][i] = data[i][4];    //Э�������ֵ
			W[i][i] = 1.0 / data[i][4];    //Э������������ֵ
		}
		Ht = mat.transposition(H);    //ת������
		HtW = mat.multiplication(Ht, W);    //�������
		Cofactor = mat.multiplication(HtW, H);
		inCofactor = mat.inverse(Cofactor);    //��������
		matrix = mat.multiplication(inCofactor, HtW);
		lsresult = mat.multiplication(matrix, Z);
		for (int i = 0; i < 4; i++)
		{
			result[i][0] =lsresult[i][0] + *(m + i);    //��ý��ջ�����λ��
		}
		delta = sqrt((*(m)-result[0][0]) * (*(m)-result[0][0]) + (*(m + 1) - result[1][0]) * (*(m + 1) - result[1][0]) + (*(m + 2) - result[2][0]) * (*(m + 2) - result[2][0]) + (*(m + 3) - result[3][0]) * (*(m + 3) - result[3][0]));    //����ǰ���ֵ
		for (int i = 0; i < 4; i++)
		{
			*(m + i) = result[i][0];    //��ֵ����Ϊ��һ�ε������
		}
		
	} while (delta > 1e-4);    //��ֵѭ��
	std::cout <<fixed<<setprecision(4)<< "x=" << result[0][0] << "\ny=" << result[1][0] << "\nz=" << result[2][0] << "\nt=" << result[3][0];
	/*�������λȨ����������*/
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
	/*���������Ʋ����������*/
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
	mat.display(varx);    //չʾ����
	/*������������*/
	allresult.push_back(result);
	vector<vector<double>> t;
	h.push_back(sigma);
	t.push_back(h);
	allresult.push_back(t);
	allresult.push_back(varx);
	return allresult;
}
