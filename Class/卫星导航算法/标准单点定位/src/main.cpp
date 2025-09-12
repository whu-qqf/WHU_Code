#include<iostream>
#include<fstream>
#include<iomanip>
#include"CoordinateConvert.h"
#include"const.h"
#include"decode.h"
#include"Matrix.h"
#include"Error.h"
#include"SatPos.h"
#include"sockets.h"
#include"SPP.h"

using namespace std;

int mode;
int main()
{
	unsigned char buff[MAXRAWLEN];    //最大缓冲区
	int lenD, lenR;    //已接收数据长度，实际接收长度
	EPOCHOBSDATA Obs;      //一个历元的观测值数据结构体
	GPSEPHREC Geph[MAXGPSNUM], Beph[MAXBDSNUM];    //一个历元星历数据结构体
	POSRES Pos,bestPos;
	FILE* FObs;
	ENUCOOR enu;
	string system;
	XYZ Userxyz,Bestxyz;
	ofstream out("ENU.txt");
	ofstream out2("Combo.txt");
	out << "week  " << "sow  " << "E  " << "N  " << "U  " <<"PDOP"<< endl;
	cout << "请输入解码模式：（0）实时数据流；（1）文件模式"<<endl;
	cin >> mode;
	if (mode == 1) {
		errno_t err = fopen_s(&FObs, "oem719-202504011900-1.bin", "rb");
		if (err != 0 || FObs == NULL)
		{
			printf("Cannot open GPS obs file. \n");
			return 0;
		}
		lenD = 0;
		while (!feof(FObs))
		{
			//fread函数参数说明：
			//1. buff+lenD：存放内存指针；
			//2. sizeof(unsigned char)：每个元素的大小；
			//3. MAXRAWLEN - lenD：要读取的元素数量；
			//4. FObs：文件指针，指向要读取的文件。
			//返回值：实际读取的元素数量，如果小于MAXRAWLEN - lenD，则表示文件已结束或发生错误。
			if ((lenR = fread(buff + lenD, sizeof(unsigned char), MAXRAWLEN - lenD, FObs)) < MAXRAWLEN - lenD) return 0;
			lenD += lenR;
			DecodeNovOem7Dat(buff, lenD, &Obs, Geph, Beph, &bestPos);   //解码会改变lenD的值，lenD为剩余未解码的数据长度
			DetectOutlier(&Obs);
			
			if(SPP(&Obs, Geph, Beph, &Pos))SPV(&Obs, &Pos);
			if (Beph->PRN != 0||Geph->PRN!=0)
			{
				Userxyz.x = Pos.Pos[0], Userxyz.y = Pos.Pos[1], Userxyz.z = Pos.Pos[2];
				Bestxyz.x = bestPos.Pos[0], Bestxyz.y = bestPos.Pos[1], Bestxyz.z = bestPos.Pos[2];
				ToENU(Bestxyz, Userxyz, enu, R_WGS84, E_WGS84);
				double distance = sqrt(pow(Bestxyz.x - Userxyz.x, 2) + pow(Bestxyz.y - Userxyz.y, 2) + pow(Bestxyz.z - Userxyz.z, 2));
				cout << fixed << setprecision(3) << "X:" << Pos.Pos[0] << " Y:" << Pos.Pos[1] << " Z:" << Pos.Pos[2] << " GPS_Clk" << Pos.GPS_clockBias << " BDS_Clk" << Pos.BDS_clockBias << " PDOP:" << Pos.PDOP << " σ:" << Pos.SigmaPos << " E:" << enu.east << " N:" << enu.north << " U:" << enu.up << " distance:" << distance << endl;
				out << fixed << setprecision(0) << Obs.Time.Week << " " << setprecision(3) << Obs.Time.SecOfWeek << "  " << enu.east << "  " << enu.north << "  " << enu.up << "  " <<Pos.PDOP<< endl;
				out2 << fixed << setprecision(0) << Obs.Time.Week << " " << setprecision(3) << Obs.Time.SecOfWeek << "  " << setprecision(0) << Obs.SatNum << endl;
				for (int i = 0; i < Obs.SatNum; i++)
				{
					if (Obs.ComObs[i].Sys == GPS)system = "G";
					else system = "C";
					out2 << system << Obs.ComObs[i].Prn << " " << setprecision(3) << Obs.ComObs[i].GF << " " << Obs.ComObs[i].MW << endl;
				}
			}
		}
		fclose(FObs);
	}
	else if (mode == 0) {
		SOCKET NetGps;
		if (OpenSocket(NetGps, "47.114.134.129", 7190) == false) {
			printf("Cannot connect to the server.\n");
		}
		lenD = 0;
		do {
			Sleep(980);
			if ((lenR = recv(NetGps, (char*)buff, MAXRAWLEN, 0)) > 0)
			{
				memcpy(buff + lenD, buff, lenR);
				lenD += lenR;
				DecodeNovOem7Dat(buff, lenD, &Obs, Geph, Beph, &bestPos);
				DetectOutlier(&Obs);
				if(SPP(&Obs, Geph, Beph, &Pos))SPV(&Obs, &Pos);
				if (Beph->PRN != 0 || Geph->PRN != 0)
				{
					Userxyz.x = Pos.Pos[0], Userxyz.y = Pos.Pos[1], Userxyz.z = Pos.Pos[2];
					Bestxyz.x = bestPos.Pos[0], Bestxyz.y = bestPos.Pos[1], Bestxyz.z = bestPos.Pos[2];
					ToENU(Bestxyz, Userxyz, enu, R_WGS84, E_WGS84);
					double distance = sqrt(pow(Bestxyz.x - Userxyz.x, 2) + pow(Bestxyz.y - Userxyz.y, 2) + pow(Bestxyz.z - Userxyz.z, 2));
					cout << fixed << setprecision(0) << Obs.Time.SecOfWeek << setprecision(3) << "X:" << Pos.Pos[0] << " Y:" << Pos.Pos[1] << " Z:" << Pos.Pos[2] << " GPS_Clk" << Pos.GPS_clockBias << " BDS_Clk" << Pos.BDS_clockBias << " PDOP:" << Pos.PDOP << " σ:" << Pos.SigmaPos << " E:" << enu.east << " N:" << enu.north << " U:" << enu.up << " distance:" << distance << endl;
					out << fixed << setprecision(0) << Obs.Time.Week << " " << setprecision(3) << Obs.Time.SecOfWeek << "  " << enu.east << "  " << enu.north << "  " << enu.up <<" "<<Pos.PDOP<< endl;
					out2 << fixed << setprecision(0) << Obs.Time.Week << " " << setprecision(3) << Obs.Time.SecOfWeek << "  " << setprecision(0) << Obs.SatNum << endl;
					for(int i=0;i<Obs.SatNum; i++) 
					{
						if (Obs.ComObs[i].Sys == GPS)system = "G";
						else system = "C";
						out2 << system << Obs.ComObs[i].Prn << " " << setprecision(3) << Obs.ComObs[i].GF << " " << Obs.ComObs[i].MW << endl;
					}
				}
			}
		} while (true);
		out.close();
		out2.close();
	}
	else {
		cout << "输入错误，请重新运行程序！" << endl;
		return -1;
	}

	return 0;
}


