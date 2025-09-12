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
/*定义接收机坐标参考真值*/
#define true_x -1.13291501648681e+6
#define true_y 6.09252850388968e+6
#define true_z 1.50463316777129e+6

int main()
{
	Readfile rfile;    //定义读取数据对象
	Matrix mat;     //定义矩阵对象
	Coordinate coor;    //定义坐标转换对象
	LS ls;    //定义最小二乘对象
	/*定义输出结果文件对象*/
	ofstream ofile;    //输出符合实验要求格式的结果
	ofstream xyzfile;     //输出各解算坐标、历元、验后单位权中误差
	ofstream enufile;     //输出转换后的坐标
	/*打开文件名*/
	ofile.open("result.txt", ios::out);
	xyzfile.open("xyz.txt", ios::out);
	enufile.open("enu.txt", ios::out);
	/*读取文件*/
	vector<vector<vector<double>>> alldata = rfile.readfiledata("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\CUSV_20212220_BDS_M0.5_I1.0_G2.0.txt");  //读取文件数据部分
	vector<vector<string>>header = rfile.readfileheader("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\CUSV_20212220_BDS_M0.5_I1.0_G2.0.txt");    //读取文件标头部分

	vector<vector<vector<double>>>allresult;    //最小二乘解算结果的三维向量，存储顺序为：解算坐标->验后单位权中误差->验后估计参数方差
	vector<vector<double>>xyz,enu;   //xyz,enu坐标
	vector<vector<double>>true_position = { {true_x},{true_y},{true_z} };    //接收机坐标参考真值
	double m[4] = { 0, 0, 0, 0 };    //解算初值数组
	xyzfile << "X(m)" << "\tY(m)" << "\tZ(m)" <<"\t t"<<"\t epoch"<< "\t sigma"<<endl;    //结果文件标头
	for (int i = 0; i < alldata.size(); i++)    //遍历传入的数据，进行第i+1个历元的解算
	{
		cout << "第"<<i+1<<"个历元"<<endl;
		cout << header[i][0]<<"   "<< header[i][1]<<"  "<< header[i][2]<<"    "<< header[i][3] <<"    "<< header[i][4] << endl;
		ofile << header[i][0] << "   " << header[i][1] << "  " << header[i][2] << "    " << header[i][3] << "    " << header[i][4] << endl;
		allresult = ls.leastsqure(m, alldata[i]);    //进行一个历元数据的最小二乘解算
		for (int j = 0; j < 4; j++)
		{
			m[j] = allresult[0][j][0];    //更新初值，将初值替换为第一个历元的最终解算结果
		}
		ofile << "X(m)" << "\tY(m)" << "\tZ(m)" << "\tT(m)" << endl;
		ofile <<fixed<<setprecision(4)<< m[0] << "    " << m[1] << "    " << m[2] << "    " << m[3] << endl;    //输出解算得到的坐标位置
		xyzfile << fixed << setprecision(4) << m[0] << "    " << m[1] << "    " << m[2] << "    " <<header[i][3]<<"    "<<i+1<<"    "<<allresult[1][0][0]<< endl;
		xyz = { {m[0]},{m[1]},{m[2]} };     //解算得到的接收机地心地固坐标系坐标
		enu = coor.XYZ2ENU(true_position, xyz);    //进行xyz->enu的坐标转换
		enufile << fixed << setprecision(4) << enu[0][0] << "    " << enu[1][0] << "    " << enu[2][0] << endl;
		enu.clear();
		ofile << "验后单位权中误差：" << allresult[1][0][0] << "(m)" << endl;    //输出验后单位权中误差
		ofile << "验后估计方差(m^2)：" << endl;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ofile <<fixed << setprecision(6)<< allresult[2][i][j]<<"    ";      //输出验后估计参数方差
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