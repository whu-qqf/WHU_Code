#pragma once
#include<vector>
#include<string>

using namespace std;

class Readfile
{
public:
	//��ȡ�ļ���������
	vector<vector<vector<double>>>readfiledata(string pathname);
	//��ȡ�ļ���ͷ����
	vector<vector<string>>readfileheader(string pathname);
};
