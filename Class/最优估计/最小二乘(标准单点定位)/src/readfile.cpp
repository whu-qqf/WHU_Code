#include<fstream>
#include<iostream>
#include<string>
#include<sstream>
#include<vector>
#include"readfile.h"

using namespace std;

vector<vector<vector<double>>> Readfile::readfiledata(string filepath)    //��ȡ�ļ�����������
{
	vector<vector<vector<double>>> alldata;    //����������Ԫ�����ݾ���
	vector<vector<double>> data;    //���嵥����Ԫ�����ݾ���
	ifstream ifile;    //��������ļ�����
	string::size_type idx;    //����string������
	ifile.open(filepath);   //���ļ�
	stringstream str;    //�ַ���������
	string line;    //һ��һ�еĶ�ȡ
	string number;    //�������Ǳ��
	double x;    //��������x
	double y;    //��������y
	double z;    //��������z
	double d;    //����α��۲�ֵd
	double var;    //�۲ⷽ��var
	int epoch=0;   //��Ԫ��
	while (getline(ifile, line))    //һ��һ�ж�ȡ
	{
		idx = line.find("#");    //Ѱ�ұ�ͷ�еķ���
		if (idx == string::npos)    //û�ҵ�
		{
			vector<double> a;
			str << line;
			str >> number >> x >> y >> z >> d >> var;   //���ݸ�ֵ
			/*װ������*/
			a.push_back(x); a.push_back(y); a.push_back(z); a.push_back(d); a.push_back(var);  
			data.push_back(a);
			str.str("");    //����ַ�����
		}
		else   //�ҵ��ˣ�˵��Ϊ��ͷ
		{
			if (epoch != 0)
			{
				alldata.push_back(data);    //�������
				data.clear();
			}
			epoch++;
			continue;
		}
	}
	alldata.push_back(data);    //������һ����Ԫ������
	ifile.close();
	return alldata;
}

vector<vector<string>>Readfile::readfileheader(string filepath)
{
	vector<vector<string>>Header;    //�����ͷ�ַ�������
	vector<string>header;  //������ͷ
	ifstream ifile;
	string::size_type idx;
	stringstream str;
	string symbol;    //��ͷ����#num
	string epoch;    //��Ԫ��
	string GPSweek;    //GPS��
	string second;    //������
	string obnum;    //�۲�ֵ����
	ifile.open(filepath);
	string line;
	while (getline(ifile, line))
	{
		idx = line.find("#");    //Ѱ�ҷ���
		if (idx == string::npos)    //û�ҵ�������
		{
			continue;
		}
		else    //�ҵ�
		{
			str << line;
			str >> symbol >> epoch >> GPSweek >> second >> obnum;    //������ֵ
			header.push_back(symbol); header.push_back(epoch); header.push_back(GPSweek); header.push_back(second); header.push_back(obnum);
			Header.push_back(header);    //�������
			header.clear();
		}
	}
	ifile.close();
	return Header;
}
