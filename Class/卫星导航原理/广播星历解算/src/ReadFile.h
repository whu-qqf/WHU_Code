#pragma once
#include<vector>
#include<string>

using namespace std;
struct Ephemeris {
	string satellitename;  //卫星名称
	int time;  //时间
	int year, month, day, hour, minute, second;  //年月日时分秒
	double a0;  //钟差
	double a1;  //钟漂
	double a2;  //钟漂率

	double iode;  //星历信息
	double Crs;  //轨道半径正弦调和改正振幅
	double delta_n;  //平均角速度改正数
	double M0;  //参考时刻的平近点角

	double Cuc; // 升交点角距余弦调和改正振幅
	double e;  //轨道偏心率
	double Cus;  //升交点角距正弦调和改正振幅
	double sqrt_A;  //轨道长半轴的平方根

	double toe;  //周内秒
	double Cic;  //轨道倾角余弦调和改正振幅
	double omega0;  //升交点赤经
	double Cis;  //轨道倾角正弦调和改正振幅

	double i0;  //轨道倾角
	double Crc;  //轨道半径余弦调和改正振幅
	double omega;  //近地点角距
	double omega_dot;  //升交点赤经变化率

	double i_dot;  //轨道倾角变化率
};

struct PreEph {
	string name;  //卫星名称
	double x;  //x坐标
	double y;  //y坐标
	double z;  //z坐标
	int time;
};

string ReadFile_Header(string filename);  //读文件头
vector<Ephemeris> ReadFile_Data(string filename);  //读文件数据
vector<PreEph> ReadFile_PreEph(string filename);  //阅读精密星历

