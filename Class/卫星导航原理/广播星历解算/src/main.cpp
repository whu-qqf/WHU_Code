#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<fstream>
#include"ReadFile.h"
#include"Calculate.h"
#include<iomanip>
using namespace std;

int main()
{
	ofstream outfile("Satellite.txt");
	string filename = "D:\\VS\\Project_VS\\C++project\\卫星定位原理\\brdm3350.19p";
	string header = ReadFile_Header(filename);
	vector<Ephemeris> alldata = ReadFile_Data(filename);
	vector<Satellite> Sat=Calculate(alldata);
	for (int i = 0; i < Sat.size(); i++)
	{
		outfile << Sat[i].satellitename <<" "<<Sat[i].time<< " " <<setiosflags(ios::fixed)<< setprecision(3) << Sat[i].x << " " << Sat[i].y << " " << Sat[i].z;
		outfile << endl;
	}
	outfile.close();

	ofstream outfile2("PreSatellite.txt");
	string spfilename = "D:\\VS\\Project_VS\\C++project\\卫星定位原理\\WUM0MGXFIN_20193350000_01D_15M_ORB.SP3";
	vector<PreEph> preeph = ReadFile_PreEph(spfilename);
	for (int i = 0; i < preeph.size(); i++)
	{
		outfile2 << preeph[i].name << " " << preeph[i].time << " " << setiosflags(ios::fixed)<<setprecision(3)<< preeph[i].x << " " << preeph[i].y << " " << preeph[i].z;
		outfile2 << endl;
	}
	outfile2.close();
	return 0;
}