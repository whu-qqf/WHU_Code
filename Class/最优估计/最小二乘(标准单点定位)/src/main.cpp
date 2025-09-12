#include<iostream>
#include<fstream>
#include<cmath>
#include<string>
#include<sstream>
#include<iomanip>
#include<vector>
#include"readfile.h"
#include"LS.h"
#include"Matrix.h"
#include"Coordinate.h"

using namespace std;
/*������ջ�����ο���ֵ*/
#define true_x -1.13291501648681e+6
#define true_y 6.09252850388968e+6
#define true_z 1.50463316777129e+6

int main()
{
	Readfile rfile;    //�����ȡ���ݶ���
	Matrix mat;     //����������
	Coordinate coor;    //��������ת������
	LS ls;    //������С���˶���
	/*�����������ļ�����*/
	ofstream ofile;    //�������ʵ��Ҫ���ʽ�Ľ��
	ofstream xyzfile;     //������������ꡢ��Ԫ�����λȨ�����
	ofstream enufile;     //���ת���������
	/*���ļ���*/
	ofile.open("result.txt", ios::out);
	xyzfile.open("xyz.txt", ios::out);
	enufile.open("enu.txt", ios::out);
	/*��ȡ�ļ�*/
	vector<vector<vector<double>>> alldata = rfile.readfiledata("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\CUSV_20212220_BDS_M0.5_I1.0_G2.0.txt");  //��ȡ�ļ����ݲ���
	vector<vector<string>>header = rfile.readfileheader("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\CUSV_20212220_BDS_M0.5_I1.0_G2.0.txt");    //��ȡ�ļ���ͷ����

	vector<vector<vector<double>>>allresult;    //��С���˽���������ά�������洢˳��Ϊ����������->���λȨ�����->�����Ʋ�������
	vector<vector<double>>xyz,enu;   //xyz,enu����
	vector<vector<double>>true_position = { {true_x},{true_y},{true_z} };    //���ջ�����ο���ֵ
	double m[4] = { 0, 0, 0, 0 };    //�����ֵ����
	xyzfile << "X(m)" << "\tY(m)" << "\tZ(m)" <<"\t t"<<"\t epoch"<< "\t sigma"<<endl;    //����ļ���ͷ
	for (int i = 0; i < alldata.size(); i++)    //������������ݣ����е�i+1����Ԫ�Ľ���
	{
		cout << "��"<<i+1<<"����Ԫ"<<endl;
		cout << header[i][0]<<"   "<< header[i][1]<<"  "<< header[i][2]<<"    "<< header[i][3] <<"    "<< header[i][4] << endl;
		ofile << header[i][0] << "   " << header[i][1] << "  " << header[i][2] << "    " << header[i][3] << "    " << header[i][4] << endl;
		allresult = ls.leastsqure(m, alldata[i]);    //����һ����Ԫ���ݵ���С���˽���
		for (int j = 0; j < 4; j++)
		{
			m[j] = allresult[0][j][0];    //���³�ֵ������ֵ�滻Ϊ��һ����Ԫ�����ս�����
		}
		ofile << "X(m)" << "\tY(m)" << "\tZ(m)" << "\tT(m)" << endl;
		ofile <<fixed<<setprecision(4)<< m[0] << "    " << m[1] << "    " << m[2] << "    " << m[3] << endl;    //�������õ�������λ��
		xyzfile << fixed << setprecision(4) << m[0] << "    " << m[1] << "    " << m[2] << "    " <<header[i][3]<<"    "<<i+1<<"    "<<allresult[1][0][0]<< endl;
		xyz = { {m[0]},{m[1]},{m[2]} };     //����õ��Ľ��ջ����ĵع�����ϵ����
		enu = coor.XYZ2ENU(true_position, xyz);    //����xyz->enu������ת��
		enufile << fixed << setprecision(4) << enu[0][0] << "    " << enu[1][0] << "    " << enu[2][0] << endl;
		enu.clear();
		ofile << "���λȨ����" << allresult[1][0][0] << "(m)" << endl;    //������λȨ�����
		ofile << "�����Ʒ���(m^2)��" << endl;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ofile <<fixed << setprecision(6)<< allresult[2][i][j]<<"    ";      //��������Ʋ�������
			}
			ofile << endl;
		}
		ofile << "------------------------------------------------------------------" << endl;
	}
	ofile.close();
	xyzfile.close();
	enufile.close();
	return 0;
}