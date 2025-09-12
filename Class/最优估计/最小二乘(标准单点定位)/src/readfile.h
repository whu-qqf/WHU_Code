#pragma once
#include<vector>
#include<string>

using namespace std;

class Readfile
{
public:
	//读取文件数据内容
	vector<vector<vector<double>>>readfiledata(string pathname);
	//读取文件标头内容
	vector<vector<string>>readfileheader(string pathname);
};
