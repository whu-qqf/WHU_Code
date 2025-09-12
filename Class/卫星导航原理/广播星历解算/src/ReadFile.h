#pragma once
#include<vector>
#include<string>

using namespace std;
struct Ephemeris {
	string satellitename;  //��������
	int time;  //ʱ��
	int year, month, day, hour, minute, second;  //������ʱ����
	double a0;  //�Ӳ�
	double a1;  //��Ư
	double a2;  //��Ư��

	double iode;  //������Ϣ
	double Crs;  //����뾶���ҵ��͸������
	double delta_n;  //ƽ�����ٶȸ�����
	double M0;  //�ο�ʱ�̵�ƽ�����

	double Cuc; // ������Ǿ����ҵ��͸������
	double e;  //���ƫ����
	double Cus;  //������Ǿ����ҵ��͸������
	double sqrt_A;  //����������ƽ����

	double toe;  //������
	double Cic;  //���������ҵ��͸������
	double omega0;  //������ྭ
	double Cis;  //���������ҵ��͸������

	double i0;  //������
	double Crc;  //����뾶���ҵ��͸������
	double omega;  //���ص�Ǿ�
	double omega_dot;  //������ྭ�仯��

	double i_dot;  //�����Ǳ仯��
};

struct PreEph {
	string name;  //��������
	double x;  //x����
	double y;  //y����
	double z;  //z����
	int time;
};

string ReadFile_Header(string filename);  //���ļ�ͷ
vector<Ephemeris> ReadFile_Data(string filename);  //���ļ�����
vector<PreEph> ReadFile_PreEph(string filename);  //�Ķ���������

