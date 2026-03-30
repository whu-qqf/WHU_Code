#include<iostream>
#include<fstream>
#include<iomanip>
#include<chrono>
#include"CoordinateConvert.h"
#include"const.h"
#include"decode.h"
#include"Matrix.h"
#include"Error.h"
#include"SatPos.h"
#include"sockets.h"
#include"SPP.h"
#include"RTK.h"
#include"Lambda.h"

using namespace std;
using namespace chrono;
int mode;
int main()
{
	unsigned char buff[MAXRAWLEN];                     /*最大缓冲区*/
	int           lenD, lenR;                          /*已接收数据长度，实际接收长度*/
	EPOCHOBSDATA  Obs;                                 /*一个历元的观测值数据结构体*/
	GPSEPHREC     Geph[MAXGPSNUM], Beph[MAXBDSNUM];    /*一个历元星历数据结构体*/
	POSRES        Pos, bestPos;                        /*定位结果数据和最佳结果数据结构体*/
	FILE*         FObs;                                /*观测文件指针*/
    ENUCOOR       enu;                                 /*东-北-天误差*/ 
	string        system;
	XYZ           Userxyz,Bestxyz;
	
	
	int SPPorRTK = 1;                                  /*0为SPP解算，1为RTK解算*/
	//mode = 1;

	/*RTK解算*/
if (SPPorRTK == 1)
	{
		cout << "请输入解码模式：（0）实时数据流；（1）文件模式" << endl;
		cin >> mode;
		cout << "全选择解算方法：（0）最小二乘法；（1）Kalman滤波" << endl;
		int Calmode = -1;
		cin >> Calmode;
		ofstream enuout("RTK_KF_0000000.txt");
		ofstream satout("sat35.txt");
		RAWDATA	      rawdata;                             /*原始数据结构体*/
		SDEPOCHOBS    SDObs;
		DDCOBS        DDObs;
		FILE* FObs_base, *FObs_rover;                                /*观测文件指针*/
		POSRES SPPPos_base, SPPPos_rover, RTKPos_Rover;
		EKFRTK EkfRTK;
		SOCKET NetGps1, NetGps2;
		enuout << "week  " << "sow  " << "E  " << "N  " << "U  " << endl;
		double EKF_X[(3 + (MAXCHANNUM - 2) * 2)] = { 0.0 };
		double EKF_P[(3 + (MAXCHANNUM - 2) * 2) * (3 + (MAXCHANNUM - 2) * 2)] = { 0.0 };
		errno_t baseerr = fopen_s(&FObs_base, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\20h-short-baseline-data\\oem719-202510311730-base.bin", "rb");
		//errno_t baseerr = fopen_s(&FObs_base, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\oem719-202510271500-base.bin", "rb");
		if (baseerr != 0 || FObs_base == NULL)
		{
			printf("Cannot open GPS obs file. \n");
			return 0;
		}
		errno_t rovererr = fopen_s(&FObs_rover, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\20h-short-baseline-data\\oem719-202510311730-rover.bin", "rb");
		//errno_t rovererr = fopen_s(&FObs_rover, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\oem719-202510271500-rover.bin", "rb");
		if (rovererr != 0 || FObs_rover == NULL)
		{
			printf("Cannot open GPS obs file. \n");
			return 0;
		}
		int status = -2;
		if (mode == 0)
		{
			if (OpenSocket(NetGps1, "8.148.22.229", 5002) == false || OpenSocket(NetGps2, "47.114.134.129", 7190) == false)
			{
				printf("Cannot connect to the server.\n");
				return -1;
			}
			unsigned long socket_mode = 1;
			if (ioctlsocket(NetGps1, FIONBIO, &socket_mode) != 0) {
				printf("Failed to set non-blocking mode. Error: %d\n", WSAGetLastError());
				CloseSocket(NetGps1);
				return 0;
			}
			if (ioctlsocket(NetGps2, FIONBIO, &socket_mode) != 0) {
				printf("Failed to set non-blocking mode. Error: %d\n", WSAGetLastError());
				CloseSocket(NetGps2);
				return 0;
			}
		}
		/*计算一次循环需要的时间*/
		auto start = high_resolution_clock::now();
		auto end = high_resolution_clock::now();
		auto duration = duration_cast<milliseconds>(end - start);
		while (true)
		{
			end = high_resolution_clock::now();
			duration = duration_cast<milliseconds>(end - start);
			if (mode == 0)
			{
				cout << "duration=" << duration.count() << endl;
				Sleep(970-duration.count());   //按照实际情况进行休眠调整
				status = RealTimeSyn(&NetGps1, &NetGps2 ,&rawdata);
				start = high_resolution_clock::now();
			}
			else if(mode==1)status = TimeSyn(FObs_base, FObs_rover, &rawdata);
			if (status == 0)continue;
			else if (status == 1)
			{	
				cout << fixed<<setprecision(0)<<"secofweek:" << rawdata.BaseObs.Time.SecOfWeek << "  ";
				DetectOutlier(&rawdata.BaseObs);
				DetectOutlier(&rawdata.RoverObs);
				if (SPP(&rawdata.BaseObs, rawdata.GPSEph, rawdata.BDSEph, &SPPPos_base))SPV(&rawdata.BaseObs, &SPPPos_base);
				else continue;
				if (SPP(&rawdata.RoverObs, rawdata.GPSEph, rawdata.BDSEph, &SPPPos_rover))SPV(&rawdata.RoverObs, &SPPPos_rover);
				else continue;
				FormSDEpochObs(&rawdata.BaseObs, &rawdata.RoverObs, &SDObs);
				DetectCirclSlip(&SDObs);
				RefSatSelection(&rawdata.BaseObs, &rawdata.RoverObs, &SDObs, &DDObs);
				int GPS_DDNum = DDObs.DDSatNums[0], BDS_DDNum = DDObs.DDSatNums[1];
				if (GPS_DDNum == 0 || BDS_DDNum == 0)if (GPS_DDNum + BDS_DDNum < 5)continue;
				else if (GPS_DDNum > 0 && BDS_DDNum > 0)if (GPS_DDNum + BDS_DDNum < 3)continue;
				satout << fixed << setprecision(3) << rawdata.BaseObs.Time.Week << " " << setprecision(3) << rawdata.BaseObs.Time.SecOfWeek << setprecision(0) <<" "<< GPS_DDNum + 1 << " " << BDS_DDNum + 1 << endl;
				double* ambi = new double[(GPS_DDNum + BDS_DDNum) * 2];
				double* Q_ambi = new double[(GPS_DDNum + BDS_DDNum) * 2 * (GPS_DDNum + BDS_DDNum) * 2];
				double* Fixed = new double[(GPS_DDNum + BDS_DDNum) * 2 * 2];
				double* s = new double[2];
				if (Calmode == 1)
				{
					if (RTK(&rawdata, &SDObs, &DDObs, &SPPPos_rover, &RTKPos_Rover) != 0)continue;

					if (EkfRTK.isInited == false) EKFINIT(&SPPPos_rover, &EkfRTK);
					TIMEPRED(&rawdata, &SDObs, &DDObs, &RTKPos_Rover, &EkfRTK);
					EKFUPDATE(&rawdata, &SDObs, &DDObs, &RTKPos_Rover, &EkfRTK);
					for (int i = 0; i < (GPS_DDNum + BDS_DDNum) * 2; i++)
					{
						ambi[i] = EkfRTK.Ambi[i];
						for (int j = 0; j < (GPS_DDNum + BDS_DDNum) * 2; j++)
						{
							Q_ambi[i * (GPS_DDNum + BDS_DDNum) * 2 + j] = EkfRTK.P[(i + 3) * (3 + (MAXCHANNUM - 2) * 2) + (j + 3)];
						}
					}
					if (lambda((GPS_DDNum + BDS_DDNum) * 2, 2, ambi, Q_ambi, Fixed, s) != 0)return -1;
					cout << "ratio=" << s[1] / s[0] << endl;
					if (s[1] / s[0] >= 3.0)
					{
						KF_FIXED2(&rawdata, &SDObs, &DDObs, &RTKPos_Rover, &EkfRTK, Fixed);
						DDObs.bFixed = true;
					}
					else if (s[1] / s[0] < 3)
					{
						DDObs.bFixed = false;
					    EkfRTK.isInited = false;
						cout << "模糊度ratio值偏低" << endl;
					}
				}
				else if (Calmode == 0)
				{
					if (RTK(&rawdata, &SDObs, &DDObs, &SPPPos_rover, &RTKPos_Rover) != 0)continue;
					for (int i = 0; i < (GPS_DDNum + BDS_DDNum) * 2; i++)
					{
						ambi[i] = DDObs.Float_Ambi[i];
						for (int j = 0; j < (GPS_DDNum + BDS_DDNum) * 2; j++)
						{
							Q_ambi[i * (GPS_DDNum + BDS_DDNum) * 2 + j] = DDObs.Qnn[(i + 3) * (3 + (MAXCHANNUM - 2) * 2) + (j + 3)];
						}
					}
					if (lambda((GPS_DDNum + BDS_DDNum) * 2, 2, ambi, Q_ambi, Fixed, s) != 0)return -1;
					double ratio = 0.0;
					cout << "ratio=" << s[1] / s[0] << endl;
					if (s[1] / s[0] >= 3)
					{
						RTK_FIXED(&rawdata, &SDObs, &DDObs, &RTKPos_Rover, Fixed);
						DDObs.bFixed = true;
					}
					else DDObs.bFixed = false;
				}
				XYZ XYZ_Rover;
				delete[] ambi, Q_ambi, Fixed, s;
				for (int i = 0; i < 3; i++)
				{
					XYZ_Rover.xyz[i] = RTKPos_Rover.Pos[i];
				}
				GEOCOOR geo;
				XYZToBLH(XYZ_Rover, geo, R_WGS84, E_WGS84);
				cout << fixed << setprecision(0) << "GPSnum:" << GPS_DDNum + 1 << " ,BDSnum:" << BDS_DDNum + 1 << setprecision(3) << "  X:" << XYZ_Rover.x << " Y:" << XYZ_Rover.y << " Z:" << XYZ_Rover.z << " dX:" << RTKPos_Rover.Pos[0] - rawdata.bestPos.Pos[0] << " dY:" << RTKPos_Rover.Pos[1] - rawdata.bestPos.Pos[1] << " dZ:" << RTKPos_Rover.Pos[2] - rawdata.bestPos.Pos[2]
					<< " PDOP:" << RTKPos_Rover.PDOP << " sigma:" << RTKPos_Rover.SigmaPos << endl;
				enuout << fixed << setprecision(3) << rawdata.BaseObs.Time.Week << " " << setprecision(3) << rawdata.BaseObs.Time.SecOfWeek
					<< setw(12) << setprecision(8) << "  " << geo.blh[0] * 180 / PI << "  " << geo.blh[1] * 180 / PI << "  " << setprecision(4) << geo.blh[2] << " " << ((DDObs.bFixed == true) ? 1 : 2)<< "  "<<setprecision(3) << RTKPos_Rover.PDOP << endl;
			}
			else if (status == -1)break;
		}
		enuout.close();
	}
	return 0;
}


