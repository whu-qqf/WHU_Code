#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include<iomanip>
#include<vector>
#include"struct.h"
#include"EKF.h"
#include"Matrix.h"
#include"const.h"

using namespace std;

int main()
{
	ifstream file_1("100COBS_2.txt");
	//ifstream file_1("100COBS_2.txt");
	ifstream file_2("100C3640.19o.gnss");
	/*ofstream outfile_5("result_5.txt");
	ofstream outfile_8("result_8.txt");
	ofstream outfile_9("result_9.txt");
	ofstream outfile_10("result_10.txt");*/
	ofstream outfile_5("result_5_error.txt");
	ofstream outfile_8("result_8_error.txt");
	//ofstream outfile_9("result_9.txt");
	//	ofstream outfile_10("result_10_error.txt");
	ofstream outfile_10("result_10_error_repair.txt");
	EKFOBS Obs[MAXCHANNUM];

	if (!file_1.is_open())
	{
		cerr << "Error opening file." << endl;
		return 1;
	}
	if (!file_2.is_open())
	{
		cerr << "Error opening file." << endl;
		return 1;
	}
	vector<ReferencePos> refPos;
	string line, PRN;
	int cur_time = -1;
	double sum_B = 0.0, sum_L = 0.0, sum_H = 0.0;
	int count = 0;
	while (getline(file_2, line))
	{
		istringstream ss(line);
		char comma;
		int week;
		double time, B, L, H;
		ss >> week >> comma >> time >> comma >> B >> comma >> L >> comma >> H;
		int sec = int(time);
		if (sec != cur_time)
		{
			if (count > 0)
			{
				refPos.push_back({ cur_time, sum_B , sum_L, sum_H });
			}
			cur_time = sec;
			sum_B = B, sum_L = L, sum_H = H;
			count = 1;
		}
	}
	if (count > 0)
	{
		refPos.push_back({ cur_time, sum_B , sum_L, sum_H});
	}
	char sig;
	double sow;
	int week, obsnum;
	double X_5[5] = { -2161303.549,4382197.209,4085792.486,9.397,20.055 };
	double D_5[5*5]={1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1 };
	double X_8[8] = { -2161303.549,4382197.209,4085792.486,0,0,0,9.397,20.055 };
	double D_8[8*8] = { 1,0,0,0,0,0,0,0,
				0,1,0,0,0,0,0,0,
				0,0,1,0,0,0,0,0,
				0,0,0,1,0,0,0,0,
				0,0,0,0,1,0,0,0,
				0,0,0,0,0,1,0,0,
				0,0,0,0,0,0,1,0,
				0,0,0,0,0,0,0,1 };
	/*double X_9[9] = { -2161303.549,4382197.209,4085792.486,9.397,20.055 ,0,0,0,0 };
	double D_9[9*9] = { 100,0,0,0,0,0,0,0,0,
				0,100,0,0,0,0,0,0,0,
				0,0,100,0,0,0,0,0,0,
				0,0,0,100,0,0,0,0,0,
				0,0,0,0,100,0,0,0,0,
				0,0,0,0,0,100,0,0,0,
				0,0,0,0,0,0,100,0,0,
				0,0,0,0,0,0,0,100,0,
		        0,0,0,0,0,0,0,0,100 };*/
	double x_10[10] = { -2161303.549,4382197.209,4085792.486,0,0,0,9.397,20.055,0,0 };
	double D_10[10*10] = { 1,0,0,0,0,0,0,0,0,0,
				0,1,0,0,0,0,0,0,0,0,
				0,0,1,0,0,0,0,0,0,0,
				0,0,0,1,0,0,0,0,0,0,
				0,0,0,0,1,0,0,0,0,0,
				0,0,0,0,0,1,0,0,0,0,
				0,0,0,0,0,0,1,0,0,0,
		        0,0,0,0,0,0,0,1,0,0,
		        0,0,0,0,0,0,0,0,1,0,
				0,0,0,0,0,0,0,0,0,1,
	};
	outfile_5 <<"t"<<" "<< "x" << " " << "y" << " " << "z" <<" "<<"GPS_clk"<<" "<<"BDS_clk" <<" "<<"E"<<" "<<" N"<<" "<<"U"<< endl;
	outfile_8 <<"t"<<" "<< "x" << " " << "y" << " " << "z" << " " << "vx" << " " << "vy" << " " << "vz" << " " << "GPS_clk" << " " << "BDS_clk" <<" "<<"E"<<" "<<"N"<<" "<<"U"<< endl;
	//outfile_9 << "t"<<" "<<"x" << " " << "y" << " " << "z" << " " << "vx" << " " << "vy" << " " << "vz" << " " << "GPS_clk" << " " << "BDS_clk" << " " << "clkshift" << endl;
	outfile_10 << "t"<<" "<<"x" << " " << "y" << " " << "z" << " " << "vx" << " " << "vy" << " " << "vz" << " " << "GPS_clk" << " " << "BDS_clk" <<" "<<"GPS_clkd"<<" "<<"BDS_clkd"<<" "<<"E"<<" "<<"N"<<" "<<"U"<< endl;
	ENUCOOR enu_5,enu_8,enu_10;

	while (getline(file_1, line))
	{
		ReferencePos* ref = new ReferencePos;
		
		if (line[0] == '#')
		{
			istringstream ss(line);
			ss >>sig>> week >> sow >> obsnum;
			for (int i = 0; i < obsnum; i++)
			{
				getline(file_1, line);
				istringstream dataStream(line);
				dataStream >> PRN >> Obs[i].X >> Obs[i].Y >> Obs[i].Z >> Obs[i].P >> Obs[i].error;
				if (PRN[0] == 'G')Obs[i].Sys = GPS;
				else if (PRN[0] == 'C')Obs[i].Sys = BDS;
				Obs[i].Prn = stoi((PRN.substr(1, 2)));
			}
			ref = &refPos[(int)sow-108143];
			//EKF_5(X_5, D_5,Obs,ref,enu_5);
			//EKF_8(X_8, D_8, Obs, ref, enu_8);
			//EKF_9(X_9, D_9, Obs, ref, enu);
			/*if ((int)sow  == 109900)
			{
				cout << "I'm here" << endl;
			}*/
			EKF_10(x_10, D_10, Obs, ref, enu_10);
			//outfile_5 << sow << " " << fixed << setprecision(3) << X_5[0] << " " << X_5[1] << " " << X_5[2] << " " << X_5[3] << " " << X_5[4] << " " << enu_5.east << " " << enu_5.north << " " << enu_5.up << endl;
			//outfile_8 << sow<<" "<<fixed << setprecision(3) << X_8[0] << " " << X_8[1] << " " << X_8[2] << " " << X_8[3] << " " << X_8[4] << " " << X_8[5] << " " << X_8[6] << " " << X_8[7] << " " << enu_8.east << " " << enu_8.north << " " << enu_8.up << endl;
			//outfile_9 << sow<<" "<<fixed << setprecision(3) << X_9[0] << " " << X_9[1] << " " << X_9[2] << " " << X_9[3] << " " << X_9[4] << " " << X_9[5] << " " << X_9[6] << " " << X_9[7] << " " << X_9[8] << endl;
			outfile_10 << sow<<" "<<fixed << setprecision(3) << x_10[0] << " " << x_10[1] << " " << x_10[2] << " " << x_10[3] << " " << x_10[4] << " " << x_10[5] << " " << x_10[6] << " " << x_10[7] << " " << x_10[8] << " " << x_10[9] << " " << enu_10.east << " " << enu_10.north << " " << enu_10.up << endl;
		}
		memset(Obs, 0, sizeof(Obs));
		//delete(ref);
	}
	file_1.close();
	outfile_5.close();
	outfile_8.close();
	//outfile_9.close();
	outfile_10.close();

	return 0;
}