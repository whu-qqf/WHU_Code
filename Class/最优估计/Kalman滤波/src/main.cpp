#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include<fstream>
#include<string>
#include<sstream>

using namespace std;

struct Obs    //�۲�ֵ�ṹ��
{
	double time;    //ʱ��
	double R;    //����
	double alpha;    //�Ƕ�
}obs[99];    //99���۲�ֵ

struct timePredict    //ʱ��Ԥ��ṹ��
{
	double time;    //ʱ��
	Eigen::MatrixXd Trans;    //״̬ת�ƾ���
	Eigen::MatrixXd X;    //״̬����
	Eigen::MatrixXd D;    //Э�������
	Eigen::MatrixXd Omega;    //��ɢ����״̬��������ʱ���޹صı���
	Eigen::MatrixXd Dw;    //״̬������
	timePredict()    //���캯����ֵ
	{
		time = 0;
		Trans = Eigen::MatrixXd(4, 4);
		X = Eigen::MatrixXd(4, 1);
		D = Eigen::MatrixXd(4, 4);
		Omega = Eigen::MatrixXd(4, 1);
		Dw = Eigen::MatrixXd(4, 4);
	}
}tp[100];

struct EKF    //��չKalman�˲�
{
	double time;    //ʱ��
	Eigen::MatrixXd obs;    //�۲�ֵ
	Eigen::MatrixXd f;    //�۲ⷽ��
	Eigen::MatrixXd V;    //��Ϣ
	Eigen::MatrixXd h;    //���Ի�����
	Eigen::MatrixXd K;    //�������
	EKF()    //���캯����ֵ
	{
		time = 0;
		obs = Eigen::MatrixXd(2, 1);
		f = Eigen::MatrixXd(2, 1);
		V = Eigen::MatrixXd(2, 1);
		h = Eigen::MatrixXd(2, 4);
		K = Eigen::MatrixXd(4, 2);
	}
}ekf[99];

struct R_update    //�۲�ֵr��Ԥ��ĸ���
{
	double time;    //ʱ��
    double V;    //��Ϣ
	Eigen::MatrixXd h;    //���Ի�����
	Eigen::MatrixXd K;    //�������
	R_update()    //���캯����ֵ
	{
		time = 0;
		V = 0;
		h = Eigen::MatrixXd(1, 4);
		K = Eigen::MatrixXd(4, 1);
	}
}R_up[99];

struct Angle_update    //�۲�ֵ�Ƕȶ�Ԥ��ĸ���
{
	double time;    //ʱ��
	double V;    //��Ϣ
	Eigen::MatrixXd h;   //���Ի�����
	Eigen::MatrixXd K;    //�������
	Angle_update()
	{
		time = 0;
		V = 0;
		h = Eigen::MatrixXd(1, 4);
		K = Eigen::MatrixXd(4, 1);
	}
}A_up[99];

void EKF(string filename)    //��չKalman�˲�����
{
	ifstream file(filename);    //���ļ�
	ofstream out_2("��չkalman�˲�.txt");    //����ļ���
	string line;    //һ��һ�еĶ�
	tp[0].time = 0.0;    //��ʼʱ��
	tp[0].X << 0, 50, 500, 0;    //��״ֵ̬
	tp[0].D << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 100, 0,
		0, 0, 0, 100;    //�۲�ֵ���̾���
	int i = 0;
	getline(file, line);
	while (getline(file, line))
	{
		/*ΪEKF�۲�ֵ��ֵ*/
		ekf[i].time = (i + 1) * 0.1;    
		istringstream iss(line);
		iss >> ekf[i].obs(0,0) >> ekf[i].obs(1,0);
		i++;
	}
	file.close();
	Eigen::Matrix2d D_delta;    //�۲��������̾���
	D_delta << 100, 0,
		0, 0.0001;
	out_2 << 0 << " " << tp[0].X(0) << " " << tp[0].X(2) << " " << tp[0].D(0, 0) << " " << tp[0].D(2, 2) << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
	//    time               x                   y                     x����			            y����            r��Ϣ      alpha��Ϣ     X����       V_x����      y����       V_y����
	for (int j = 1; j < 100; j++)    //ѭ������
	{
		tp[j].time = j * 0.1;
		tp[j].Trans << 1, 0.1, 0, 0,
			0, 1 - 2 * 0.01 * tp[j - 1].X(1) * 0.1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1 + 2 * 0.05 * tp[j - 1].X(3) * 0.1;    //״̬ת�ƾ���
		tp[j].Omega << 0.5 * 0.1 * 0.1 * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			(0.1 - 0.01 * tp[j - 1].X(1) * 0.01) * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			-0.5 * 0.01 * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8),
			-(0.1 + 0.05 * tp[j - 1].X(3) * 0.01) * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8);    //��ɢ����״̬������ʱ���޹ص���
		tp[j].Dw << 0.001 * 2.25 / 3, (0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, 0, 0,
			(0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, (0.3 - 6 * 0.01 * tp[j - 1].X(1) * 0.01 + 4 * 0.01 * tp[j - 1].X(1) * 0.01 * tp[j - 1].X(1) * 0.001) * 2.25 / 3, 0, 0,
			0, 0, 0.001 * 2.25 / 3, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25,
			0, 0, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25, (0.3 + 6 * 0.05 * tp[j - 1].X(3) * 0.01 + 4 * 0.05 * tp[j - 1].X(3) * 0.05 * tp[j - 1].X(3) * 0.001) * 2.25 / 3;    //״̬����������
		//����ʱ��Ԥ��
		tp[j].X = tp[j].Trans * tp[j - 1].X + tp[j].Omega;
		tp[j].D = tp[j].Trans * tp[j - 1].D * tp[j].Trans.inverse() + tp[j].Dw;
		ekf[j - 1].f << sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			atan2(tp[j].X(0), tp[j].X(2));    //�۲����
		ekf[j - 1].V = ekf[j - 1].obs - ekf[j - 1].f;    //��Ϣ����
		ekf[j - 1].h << tp[j].X(0) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0, tp[j].X(2) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0,
			tp[j].X(2) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0, -tp[j].X(0) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)), 0;    //�۲ⷽ�����Ի�����
		ekf[j - 1].K = tp[j].D * ekf[j - 1].h.transpose() * (ekf[j - 1].h * tp[j].D * ekf[j - 1].h.transpose() + D_delta).inverse();    //�������
		//���в�������
		tp[j].X = tp[j].X + ekf[j - 1].K * ekf[j - 1].V;
		tp[j].D = tp[j].D - ekf[j - 1].K * ekf[j - 1].h * tp[j].D;
		out_2 << tp[j].time << " " << tp[j].X(0) << " " << tp[j].X(2) << " " << tp[j].D(0, 0) << " " << tp[j].D(2, 2) << " "
			<< ekf[j - 1].V(0, 0) << " " << ekf[j - 1].V(1, 0) << " "
			<< (ekf[j - 1].K * ekf[j - 1].V)(0, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(1, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(2, 0) << " " << (ekf[j - 1].K * ekf[j - 1].V)(3, 0) << endl;    //������
	}
	out_2.close();
}
void Obs_EKF(string filename)    //�۲�ֵ��θ��µ���չKalman�˲�
{
	ifstream file(filename);
	ofstream out_1("�۲�ֵ��θ��µ���չkalman�˲�.txt");
	string line;
	tp[0].time = 0.0;
	tp[0].X << 0, 50, 500, 0;    //��״ֵ̬
	tp[0].D << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 100, 0,
		0, 0, 0, 100;    //��ʼ״̬�������
	int i = 0;
	getline(file, line);
	while (getline(file, line))     //��������
	{
		obs[i].time = (i + 1) * 0.1;
		istringstream iss(line);
		iss >> obs[i].R >> obs[i].alpha;    //�۲�ֵ�洢
		i++;
	}
	file.close();
	out_1 << 0 << " " << tp[0].X(0) << " " << tp[0].X(2) << " "     /*ʱ��    x     y*/
		<< tp[0].D(0, 0) << " " << tp[0].D(2, 2) << " "    /*x����    y����*/
		<< 0 << " " << 0 << " "     /*�����۲�ֵ����Ϣ*/
		<< 0 << " " << 0 << " " << 0 << " " << 0 << " "    /*r�۲�ֵ��״̬������*/
		<< 0 << " " << 0 << " " << 0 << " " << 0 << endl;    //�Ƕȹ۲�ֵ��״̬������
	for (int j = 1; j < 100; j++)
	{
		tp[j].time = j * 0.1;
		tp[j].Trans << 1, 0.1, 0, 0,
			0, 1 - 2 * 0.01 * tp[j - 1].X(1) * 0.1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1 + 2 * 0.05 * tp[j - 1].X(3) * 0.1;     //״̬ת�ƾ���
		tp[j].Omega << 0.5 * 0.1 * 0.1 * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			(0.1 - 0.01 * tp[j - 1].X(1) * 0.01) * 0.01 * tp[j - 1].X(1) * tp[j - 1].X(1),
			-0.5 * 0.01 * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8),
			-(0.1 + 0.05 * tp[j - 1].X(3) * 0.01) * (0.05 * tp[j - 1].X(3) * tp[j - 1].X(3) + 9.8);    //��ɢ����״̬���̷������
		tp[j].Dw << 0.001 * 2.25 / 3, (0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, 0, 0,
			(0.01 * 0.5 - 0.002 * 0.01 * tp[j - 1].X(1) / 3) * 2.25, (0.3 - 6 * 0.01 * tp[j - 1].X(1) * 0.01 + 4 * 0.01 * tp[j - 1].X(1) * 0.01 * tp[j - 1].X(1) * 0.001) * 2.25 / 3, 0, 0,
			0, 0, 0.001 * 2.25 / 3, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25,
			0, 0, (0.01 * 0.5 + 0.002 * 0.05 * tp[j - 1].X(3) / 3) * 2.25, (0.3 + 6 * 0.05 * tp[j - 1].X(3) * 0.01 + 4 * 0.05 * tp[j - 1].X(3) * 0.05 * tp[j - 1].X(3) * 0.001) * 2.25 / 3;    //״̬����������
		//����ʱ��Ԥ��
		tp[j].X = tp[j].Trans * tp[j - 1].X + tp[j].Omega;
		tp[j].D = tp[j].Trans * tp[j - 1].D * tp[j].Trans.inverse() + tp[j].Dw;

		R_up[j - 1].V = obs[j - 1].R - sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2));    //r�۲�ֵ����Ϣ
		R_up[j - 1].h << tp[j].X(0) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),    
			0,
			tp[j].X(2) / sqrt(tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0;    //r�۲�ֵ���Ի�����
		R_up[j - 1].K = tp[j].D * R_up[j - 1].h.transpose() / ((R_up[j - 1].h * tp[j].D * R_up[j - 1].h.transpose())(0) + 100);    //r���������
		//�۲�ֵr��ʱ��Ԥ��ĸ���
		tp[j].X = tp[j].X + R_up[j - 1].K * R_up[j - 1].V;    
		tp[j].D = tp[j].D - R_up[j - 1].K * R_up[j - 1].h * tp[j].D;

		A_up[j - 1].V = obs[j - 1].alpha - atan2(tp[j].X(0), tp[j].X(2));    //�Ƕȵ���Ϣ
		A_up[j - 1].h << tp[j].X(2) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0,
			-tp[j].X(0) / (tp[j].X(0) * tp[j].X(0) + tp[j].X(2) * tp[j].X(2)),
			0;    //�Ƕȹ۲�ֵ���Ի�����
		A_up[j - 1].K = tp[j].D * A_up[j - 1].h.transpose() / ((A_up[j - 1].h * tp[j].D * A_up[j - 1].h.transpose())(0) + 0.0001);    //�������
		//�۲�ֵ����ʱ��Ԥ��ĸ���
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
	cout << "�������!" << endl;
	return 0;
}
