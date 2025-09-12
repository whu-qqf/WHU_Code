#include<fstream>
#include<iostream>
#include<string>
#include<sstream>
#include<vector>
#include"readfile.h"

using namespace std;

vector<vector<vector<double>>> Readfile::readfiledata(string filepath)    //读取文件的数据内容
{
	vector<vector<vector<double>>> alldata;    //定义所有历元的数据矩阵
	vector<vector<double>> data;    //定义单个历元的数据矩阵
	ifstream ifile;    //定义读入文件对象
	string::size_type idx;    //定义string的索引
	ifile.open(filepath);   //打开文件
	stringstream str;    //字符串流对象
	string line;    //一行一行的读取
	string number;    //定义卫星编号
	double x;    //卫星坐标x
	double y;    //卫星坐标y
	double z;    //卫星坐标z
	double d;    //卫星伪距观测值d
	double var;    //观测方差var
	int epoch=0;   //历元数
	while (getline(ifile, line))    //一行一行读取
	{
		idx = line.find("#");    //寻找标头中的符号
		if (idx == string::npos)    //没找到
		{
			vector<double> a;
			str << line;
			str >> number >> x >> y >> z >> d >> var;   //数据赋值
			/*装入数据*/
			a.push_back(x); a.push_back(y); a.push_back(z); a.push_back(d); a.push_back(var);  
			data.push_back(a);
			str.str("");    //清空字符串流
		}
		else   //找到了，说明为标头
		{
			if (epoch != 0)
			{
				alldata.push_back(data);    //存放数据
				data.clear();
			}
			epoch++;
			continue;
		}
	}
	alldata.push_back(data);    //存放最后一个历元的数据
	ifile.close();
	return alldata;
}

vector<vector<string>>Readfile::readfileheader(string filepath)
{
	vector<vector<string>>Header;    //定义标头字符串矩阵
	vector<string>header;  //单个标头
	ifstream ifile;
	string::size_type idx;
	stringstream str;
	string symbol;    //标头符号#num
	string epoch;    //历元数
	string GPSweek;    //GPS周
	string second;    //周内秒
	string obnum;    //观测值数量
	ifile.open(filepath);
	string line;
	while (getline(ifile, line))
	{
		idx = line.find("#");    //寻找符号
		if (idx == string::npos)    //没找到，跳过
		{
			continue;
		}
		else    //找到
		{
			str << line;
			str >> symbol >> epoch >> GPSweek >> second >> obnum;    //变量赋值
			header.push_back(symbol); header.push_back(epoch); header.push_back(GPSweek); header.push_back(second); header.push_back(obnum);
			Header.push_back(header);    //存放数据
			header.clear();
		}
	}
	ifile.close();
	return Header;
}
