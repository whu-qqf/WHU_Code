#include"ReadFile.h"
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>

#include<iomanip>
#include<map>
#include<set>
#include<algorithm>

using namespace std;

string ReadFile_Header(string filename)
{
	ifstream infile;
	infile.open(filename);
	if (!infile.is_open())
	{
		cout << "文件打开失败" << endl;
		return "error";
	}
	stringstream header_o;
	string header;
	string str;
	while (getline(infile, str))
	{
		header_o << str <<'\n';
		if (str.find("END OF HEADER") != string::npos) break;
	}
	infile.close();
	header = header_o.str();
	return header;
}	


vector<Ephemeris> ReadFile_Data(string filename)
{
	ifstream infile;
	string satellitename;
	infile.open(filename);
	if (!infile.is_open())
	{
		cout << "文件打开失败" << endl;
		return vector<Ephemeris>();
	}
	vector<Ephemeris> alldata;
	string line;
	while (getline(infile, line))
	{
		if (line.find("END OF HEADER") != string::npos)break;
	}
	map<string, vector<Ephemeris>> satellite_map; 
	set<string> satellite_set;
	while (getline(infile, line))
	{
		unsigned __int64 index = line.find("-");
		while (index != string::npos)
		{
			if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
			{
				line.insert(index, " ");
				index = line.find("-", index + 2); // 跳过插入的空格和负号
			}
			else
			{
				index = line.find("-", index + 1); // 继续查找下一个负号
			}
		}
		stringstream ss(line);
		if (line.find("G") != string::npos || line.find("C") != string::npos)
		{
			Ephemeris* eph = new Ephemeris();
			ss >> eph->satellitename >> eph->year >> eph->month >> eph->day >> eph->hour >> eph->minute >> eph->second
				>> eph->a0 >> eph->a1 >> eph->a2;
			eph->time = eph->hour * 3600 + eph->minute * 60 + eph->second;
			ss.clear();
			getline(infile, line);
			while (index != string::npos)
			{
				if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
				{
					line.insert(index, " ");
					index = line.find("-", index + 2); // 跳过插入的空格和负号
				}
				else
				{
					index = line.find("-", index + 1); // 继续查找下一个负号
				}
			}
			ss.str(line);
			ss >> eph->iode >> eph->Crs >> eph->delta_n >> eph->M0;
			ss.clear();
			getline(infile, line);
			while (index != string::npos)
			{
				if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
				{
					line.insert(index, " ");
					index = line.find("-", index + 2); // 跳过插入的空格和负号
				}
				else
				{
					index = line.find("-", index + 1); // 继续查找下一个负号
				}
			}
			ss.str(line);
			ss >> eph->Cuc >> eph->e >> eph->Cus >> eph->sqrt_A;
			ss.clear();
			getline(infile, line);
			while (index != string::npos)
			{
				if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
				{
					line.insert(index, " ");
					index = line.find("-", index + 2); // 跳过插入的空格和负号
				}
				else
				{
					index = line.find("-", index + 1); // 继续查找下一个负号
				}
			}
			ss.str(line);
			ss >> eph->toe >> eph->Cic >> eph->omega0 >> eph->Cis;
			ss.clear();
			getline(infile, line);
			while (index != string::npos)
			{
				if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
				{
					line.insert(index, " ");
					index = line.find("-", index + 2); // 跳过插入的空格和负号
				}
				else
				{
					index = line.find("-", index + 1); // 继续查找下一个负号
				}
			}
			ss.str(line);
			ss >> eph->i0 >> eph->Crc >> eph->omega >> eph->omega_dot;
			ss.clear();
			getline(infile, line);
			while (index != string::npos)
			{
				if (index == 0 || line[index - 1] != 'e') // 检查负号前是否有 'e'
				{
					line.insert(index, " ");
					index = line.find("-", index + 2); // 跳过插入的空格和负号
				}
				else
				{
					index = line.find("-", index + 1); // 继续查找下一个负号
				}
			}
			ss.str(line);
			ss >> eph->i_dot;
			satellite_map[eph->satellitename].push_back(*eph); // 将星历数据存储到 map 中
			satellite_set.insert(eph->satellitename); // 将卫星名称存储到 set 中
			delete eph;
		}
	}
	infile.close();
	for (string t : satellite_set)
	{
		vector<Ephemeris> data = satellite_map[t];
		sort(data.begin(), data.end(), [](const Ephemeris& a, const Ephemeris& b) {
			return a.time < b.time; // 按时间升序排序
		});
		for (int i = 0; i <= 86400; i++)
		{
			int closest_time = data[0].time;
			int min_dist = abs(data[0].time - i);
			for (const auto& p : data)
			{
				int dist = abs(p.time - i);
				if (dist < min_dist || dist == min_dist && p.time < closest_time)
				{
					min_dist = dist;
					closest_time = p.time;
				}
			}
			auto found = find_if(data.begin(), data.end(),
				[closest_time](const auto& p) { return p.time == closest_time; });
			if (found != data.end())
			{
				Ephemeris* eph = new Ephemeris();
				eph->satellitename = found->satellitename;
				eph->time = i;
				eph->a0 = found->a0;
				eph->a1 = found->a1;
				eph->a2 = found->a2;
				eph->iode = found->iode;
				eph->Crs = found->Crs;
				eph->delta_n = found->delta_n;
				eph->M0 = found->M0;
				eph->Cuc = found->Cuc;
				eph->e = found->e;
				eph->Cus = found->Cus;
				eph->sqrt_A = found->sqrt_A;
				eph->toe = found->toe;
				eph->Cic = found->Cic;
				eph->omega0 = found->omega0;
				eph->Cis = found->Cis;
				eph->i0 = found->i0;
				eph->Crc = found->Crc;
				eph->omega = found->omega;
				eph->omega_dot = found->omega_dot;
				eph->i_dot = found->i_dot;
				alldata.push_back(*eph);
				delete eph;
			}
		}
	}
	return alldata;



}

vector<PreEph> ReadFile_PreEph(string filename)
{
	ifstream infile;
	infile.open(filename);
	string star;
	int t;
	int year, month, day, hour, minute, second;
	if (!infile.is_open())
	{
		cout << "文件打开失败" << endl;
		return vector<PreEph>();
	}
	vector<PreEph> alldata;
	string line;
	for (int i = 0; i < 32; i++)
	{
		getline(infile, line);
	}
	while (getline(infile, line))
	{
		stringstream ss(line);
		if (line.find("*") != string::npos)
		{
			ss >> star >> year >> month >> day >> hour >> minute >> second;
			t = hour * 3600 + minute * 60 + second;
			ss.clear();
		}
		if (line.find("G") != string::npos || line.find("C") != string::npos)
		{
			size_t index = line.find("P");
			if (index != string::npos)
			{
				line.erase(index, 1);
			}
			ss.str(line);
			PreEph* preeph = new PreEph();

			preeph->time = t;
			ss >> preeph->name >> preeph->x >> preeph->y >> preeph->z;
			preeph ->x *= 1000;
			preeph->y *= 1000;
			preeph->z *= 1000;
			alldata.push_back(*preeph);
			ss.clear();
			delete preeph;
		}
	}
	infile.close();
	return alldata;
}