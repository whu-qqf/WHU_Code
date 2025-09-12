#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include<fstream>
#include<string>
#include<sstream>

using namespace std;

struct Obs    //观测值结构体
{
	double time;    //时间
	double R;    //距离
	double alpha;    //角度
}obs[99];    //99个观测值

struct timePredict    //时间预测结构体
{
	double time;    //时间
	Eigen::MatrixXd Trans;    //状态转移矩阵
	Eigen::MatrixXd X;    //状态变量
	Eigen::MatrixXd D;    //协方差矩阵
	Eigen::MatrixXd Omega;    //离散化后状态方程里与时间无关的变量
	Eigen::MatrixXd Dw;    //状态误差方差阵
	timePredict()    //构造函数赋值
	{
		time = 0;
		Trans = Eigen::MatrixXd(4, 4);
		X = Eigen::MatrixXd(4, 1);
		D = Eigen::MatrixXd(4, 4);
		Omega = Eigen::MatrixXd(4, 1);
		Dw = Eigen::MatrixXd(4, 4);
	}
}tp[100];

struct EKF    //扩展Kalman滤波
{
	double time;    //时间
	Eigen::MatrixXd obs;    //观测值
	Eigen::MatrixXd f;    //观测方程
	Eigen::MatrixXd V;    //新息
	Eigen::MatrixXd h;    //线性化矩阵
	Eigen::MatrixXd K;    //增益矩阵
	EKF()    //构造函数赋值
	{
		time = 0;
		obs = Eigen::MatrixXd(2, 1);
		f = Eigen::MatrixXd(2, 1);
		V = Eigen::MatrixXd(2, 1);
		h = Eigen::MatrixXd(2, 4);
		K = Eigen::MatrixXd(4, 2);
	}
}ekf[99];

struct R_update    //观测值r对预测的更新
{
	double time;    //时间
    double V;    //新息
	Eigen::MatrixXd h;    //线性化矩阵
	Eigen::MatrixXd K;    //增益矩阵
	R_update()    //构造函数赋值
	{
		time = 0;
		V = 0;
		h = Eigen::MatrixXd(1, 4);
		K = Eigen::MatrixXd(4, 1);
	}
}R_up[99];

struct Angle_update    //观测值角度对预测的更新
{
	double time;    //时间
	double V;    //新息
	Eigen::MatrixXd h;   //线性化矩阵
	Eigen::MatrixXd K;    //增益矩阵
	Angle_update()
	{
		time = 0;
		V = 0;
		h = Eigen::MatrixXd(1, 4);
		K = Eigen::MatrixXd(4, 1);
	}
}A_up[99];

void EKF(string filename)    //扩展Kalman滤波解算
{
	ifstream file(filename);    //打开文件
	ofstream out_2("扩展kalman滤波.txt");    //输出文件名
	string line;    //一行一行的读
	tp[0].time = 0.0;    //初始时间
	tp[0].X << 0, 50, 500, 0;    //初状态值
	tp[0].D << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 100, 0,
		0, 0, 0, 100;    //观测值方程矩阵
	int i = 0;
	getline(file, line);
	while (getline(file, line))
	{
		/*为EKF观测值赋值*/
		ekf[i].time = (i + 1) * 0.1;    
		istringstream iss(line);
		iss >> ekf[i].obs(0,0) >> ekf[i].obs(1,0);
		i++;
	}
	file.close();
	Eigen::Matrix2d D_delta;    //观测噪声方程矩阵
	D_delta << 100, 0,
		0, 0.0001;
	out_2 << 0 << " " << tp[0].X(0) << " " << tp[0].X(2) << " " << tp[0].D(0, 0) << " " << tp[0].D(2, 2) << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
	//    time               x                   y                     x方差			            y方差            r新息      alpha新息     X增益       V_x增益      y增益       V_y增益
	for (int j = 1; j < 100; j++)    //循环解算
	{
		tp[j].time = j * 0.1;
		tp[j].Trans << 1, 0.1, 0, 0,
			0, 1 - 2 * 0.01 * tp[j - 1].X(1) * 0.1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1 + 2 * 0.05 * tp[j - 1].X(3) * 0.1;    //状态转移矩阵
		tp[j].Omega << 0.5 * 0.1 * 0.1 * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			(0.1 - 0.01 * tp[j - 1].X(1) * 0.01) * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			-0.5 * 0.01 * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8),
			-(0.1 + 0.05 * tp[j - 1].X(3) * 0.01) * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8);    //离散化后状态方程与时间无关的量
		tp[j].Dw << 0.001 * 2.25 / 3, (0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, 0, 0,
			(0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, (0.3 - 6 * 0.01 * tp[j - 1].X(1) * 0.01 + 4 * 0.01 * tp[j - 1].X(1) * 0.01 * tp[j - 1].X(1) * 0.001) * 2.25 / 3, 0, 0,
			0, 0, 0.001 * 2.25 / 3, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25,
			0, 0, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25, (0.3 + 6 * 0.05 * tp[j - 1].X(3) * 0.01 + 4 * 0.05 * tp[j - 1].X(3) * 0.05 * tp[j - 1].X(3) * 0.001) * 2.25 / 3;    //状态方程误差方差阵
		//进行时间预测
		tp[j].X = tp[j].Trans * tp[j - 1].X + tp[j].Omega;
		tp[j].D = tp[j].Trans * tp[j - 1].D * tp[j].Trans.inverse() + tp[j].Dw;
		ekf[j - 1].f << sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			atan2(tp[j].X(0), tp[j].X(2));    //观测矩阵
		ekf[j - 1].V = ekf[j - 1].obs - ekf[j - 1].f;    //新息矩阵
		ekf[j - 1].h << tp[j].X(0) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0, tp[j].X(2) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0,
			tp[j].X(2) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0, -tp[j].X(0) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0;    //观测方程线性化矩阵
		ekf[j - 1].K = tp[j].D * ekf[j - 1].h.transpose() * (ekf[j - 1].h * tp[j].D * ekf[j - 1].h.transpose() + D_delta).inverse();    //增益矩阵
		//进行测量更新
		tp[j].X = tp[j].X + ekf[j - 1].K * ekf[j - 1].V;
		tp[j].D = tp[j].D - ekf[j - 1].K * ekf[j - 1].h * tp[j].D;
		out_2 << tp[j].time << " " << tp[j].X(0) << " " << tp[j].X(2) << " " << tp[j].D(0, 0) << " " << tp[j].D(2, 2) << " "
			<< ekf[j - 1].V(0, 0) << " " << ekf[j - 1].V(1, 0) << " "
			<< (ekf[j - 1].K * ekf[j - 1].V)(0, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(1, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(2, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(3, 0) << endl;    //结果输出
	}
	out_2.close();
}
void Obs_EKF(string filename)    //观测值逐次更新的扩展Kalman滤波
{
	ifstream file(filename);
	ofstream out_1("观测值逐次更新的扩展kalman滤波.txt");
	string line;
	tp[0].time = 0.0;
	tp[0].X << 0, 50, 500, 0;    //初状态值
	tp[0].D << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 100, 0,
		0, 0, 0, 100;    //初始状态方差矩阵
	int i = 0;
	getline(file, line);
	while (getline(file, line))     //读入数据
	{
		obs[i].time = (i + 1) * 0.1;
		istringstream iss(line);
		iss >> obs[i].R >> obs[i].alpha;    //观测值存储
		i++;
	}
	file.close();
	out_1 << 0 << " " << tp[0].X(0) << " " << tp[0].X(2) << " "     /*时间    x     y*/
		<< tp[0].D(0, 0) << " " << tp[0].D(2, 2) << " "    /*x方差    y方差*/
		<< 0 << " " << 0 << " "     /*两个观测值的新息*/
		<< 0 << " " << 0 << " " << 0 << " " << 0 << " "    /*r观测值对状态的增益*/
		<< 0 << " " << 0 << " " << 0 << " " << 0 << endl;    //角度观测值对状态的增益
	for (int j = 1; j < 100; j++)
	{
		tp[j].time = j * 0.1;
		tp[j].Trans << 1, 0.1, 0, 0,
			0, 1 - 2 * 0.01 * tp[j - 1].X(1) * 0.1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1 + 2 * 0.05 * tp[j - 1].X(3) * 0.1;     //状态转移矩阵
		tp[j].Omega << 0.5 * 0.1 * 0.1 * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			(0.1 - 0.01 * tp[j - 1].X(1) * 0.01) * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			-0.5 * 0.01 * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8),
			-(0.1 + 0.05 * tp[j - 1].X(3) * 0.01) * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8);    //离散化后状态方程非随机量
		tp[j].Dw << 0.001 * 2.25 / 3, (0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, 0, 0,
			(0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, (0.3 - 6 * 0.01 * tp[j - 1].X(1) * 0.01 + 4 * 0.01 * tp[j - 1].X(1) * 0.01 * tp[j - 1].X(1) * 0.001) * 2.25 / 3, 0, 0,
			0, 0, 0.001 * 2.25 / 3, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25,
			0, 0, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25, (0.3 + 6 * 0.05 * tp[j - 1].X(3) * 0.01 + 4 * 0.05 * tp[j - 1].X(3) * 0.05 * tp[j - 1].X(3) * 0.001) * 2.25 / 3;    //状态方程误差方差阵
		//进行时间预测
		tp[j].X = tp[j].Trans * tp[j - 1].X + tp[j].Omega;
		tp[j].D = tp[j].Trans * tp[j - 1].D * tp[j].Trans.inverse() + tp[j].Dw;

		R_up[j - 1].V = obs[j - 1].R - sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2));    //r观测值的新息
		R_up[j - 1].h << tp[j].X(0) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),    
			0,
			tp[j].X(2) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0;    //r观测值线性化矩阵
		R_up[j - 1].K = tp[j].D * R_up[j - 1].h.transpose() / ((R_up[j - 1].h * tp[j].D * R_up[j - 1].h.transpose())(0) + 100);    //r的增益矩阵
		//观测值r对时间预测的更新
		tp[j].X = tp[j].X + R_up[j - 1].K * R_up[j - 1].V;    
		tp[j].D = tp[j].D - R_up[j - 1].K * R_up[j - 1].h * tp[j].D;

		A_up[j - 1].V = obs[j - 1].alpha - atan2(tp[j].X(0), tp[j].X(2));    //角度的新息
		A_up[j - 1].h << tp[j].X(2) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0,
			-tp[j].X(0) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0;    //角度观测值线性化矩阵
		A_up[j - 1].K = tp[j].D * A_up[j - 1].h.transpose() / ((A_up[j - 1].h * tp[j].D * A_up[j - 1].h.transpose())(0) + 0.0001);    //增益矩阵
		//观测值α对时间预测的更新
		tp[j].X = tp[j].X + A_up[j - 1].K * A_up[j - 1].V;
		tp[j].D = tp[j].D - A_up[j - 1].K * A_up[j - 1].h * tp[j].D;
		out_1 << tp[j].time << " " << tp[j].X(0) << " " << tp[j].X(2) << " " << tp[j].D(0, 0) << " " << tp[j].D(2, 2) << " " 
			<< R_up[j - 1].V << " " << A_up[j - 1].V << " " 
			<< (R_up[j - 1].K * R_up[j - 1].V)(0, 0) << " " << (R_up[j - 1].K * R_up[j - 1].V)(1, 0) << " " << (R_up[j - 1].K * R_up[j - 1].V)(2, 0) << " " << (R_up[j - 1].K * R_up[j - 1].V)(3, 0) << " "
			<< (A_up[j - 1].K * A_up[j - 1].V)(0, 0) << " " << (A_up[j - 1].K * A_up[j - 1].V)(1, 0) << " " << (A_up[j - 1].K * A_up[j - 1].V)(2, 0) << " " << (A_up[j - 1].K * A_up[j - 1].V)(3, 0) << endl;
	}
	out_1.close();
}


int main()
{
	string filename = "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\observation.txt";
	Obs_EKF(filename);
	EKF(filename);
	cout << "结算完成!" << endl;
	return 0;
}
