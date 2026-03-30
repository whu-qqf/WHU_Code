#include<windows.h>
#include<iostream>
#include<string>
#include<fstream>
#include<iomanip>
#include<vector>
#include<memory>
#include"IMU.h"
#include"Decode.h"
#include"Matrix.h"
using namespace std;
int main()
{
	int CalMode = -1;
	
	cout << "请选择：（0）IMU标定；（1）静态粗对准；（2）示例程序验证；（3）惯导解算" << endl;
	cin >> CalMode;
	IMU_ERROR imu_error;

	if (CalMode == 0)
	{
		IMU_DATA x_down, x_up, y_down, y_up, z_down, z_up;
		IMU_DATA x_fwd, x_rev, y_fwd, y_rev, z_fwd, z_rev;
		ofstream xup, xdown, yup, ydown, zup, zdown, xfwd, xrev, yfwd, yrev, zfwd, zrev;
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\x_down.ASC", &x_down);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\x_up.ASC", &x_up);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\y_down.ASC", &y_down);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\y_up.ASC", &y_up);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\z_down.ASC", &z_down);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\加速度计标定\\z_up.ASC", &z_up);
	    IMU_DATA data_set[6] = { x_up,  x_down,  y_up,  y_down,  z_up,  z_down };
		std::cout << "数据已经读取完成...加速度计标定即将开始解算。" << endl;
		ACC_Cal(data_set, &imu_error);
		cout << "M矩阵为:" << endl;
		PrintMatrix(3, 4, imu_error.M, 12, 8);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\x_FWD.ASC", &x_fwd);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\x_REV.ASC", &x_rev);
		GYR_Cal(&x_fwd, &x_rev, 0, &imu_error);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\y_FWD.ASC", &y_fwd);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\y_REV.ASC", &y_rev);
		GYR_Cal(&y_fwd, &y_rev, 1, &imu_error);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\z_FWD.ASC", &z_fwd);
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IMU\\标定\\3\\asc\\陀螺仪标定\\z_REV.ASC", &z_rev);
		GYR_Cal(&z_fwd, &z_rev, 2, &imu_error);
		int isUse = -1;
		cout << "是否利用加速度计六位置法静态数据标定陀螺零偏：（0）利用；（1）不利用" << endl;
		cin >> isUse;
		if (isUse == 0)GYR_Bias(data_set, &imu_error);
		cout << "陀螺仪零偏和比例因子误差矩阵:" << endl;
		PrintMatrix(3, 4, imu_error.Gyo, 12, 8);
		cout << "三轴零偏：" << endl;
		for (int i = 0; i < 3; i++)
		{
			cout << fixed << setprecision(8) << imu_error.Gyo[4 * i + 3] << endl;
		}
		cout << "三轴比例因子：" << endl;
		for (int i = 0; i < 3; i++)
		{
			cout << fixed << setprecision(8) << imu_error.Gyo[4 * i + i] - 1 << endl;
		}
		/*cout << "正在进行数据补偿..." << endl;
		Compensation(&x_up, &imu_error);
		Compensation(&x_down, &imu_error);
		Compensation(&y_up, &imu_error);
		Compensation(&y_down, &imu_error);
		Compensation(&z_up, &imu_error);
		Compensation(&z_down, &imu_error);
		Compensation(&x_fwd, &imu_error);
		Compensation(&x_rev, &imu_error);
		Compensation(&y_fwd, &imu_error);
		Compensation(&y_rev, &imu_error);
		Compensation(&z_fwd, &imu_error);
		Compensation(&z_rev, &imu_error);*/
		/*cout << "数据补偿完成，正在保存数据..." << endl;
		xup.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\x_up-补偿后.ASC");
		xdown.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\x_down-补偿后.ASC");
		yup.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\y_up-补偿后.ASC");
		ydown.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\y_down-补偿后.ASC");
		zup.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\z_up-补偿后.ASC");
		zdown.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\加速度计标定\\z_down-补偿后.ASC");
		xfwd.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\x_FWD-补偿后.ASC");
		xrev.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\x_REV-补偿后.ASC");
		yfwd.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\y_FWD-补偿后.ASC");
		yrev.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\y_REV-补偿后.ASC");
		zfwd.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\z_FWD-补偿后.ASC");
		zrev.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\3\\asc\\陀螺仪标定\\z_REV-补偿后.ASC");
		for (size_t i = 0; i < x_up.secofweek.size(); i++)
		{
			xup << x_up.week[i] << " " << fixed << setprecision(8) << x_up.secofweek[i] << " " << x_up.x_acc[i] << " " << x_up.y_acc[i] << " " << x_up.z_acc[i] << " " << x_up.x_gyo[i] << " " << x_up.y_gyo[i] << " " << x_up.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < x_down.secofweek.size(); i++)
		{
			xdown << x_down.week[i] << " " << fixed << setprecision(8) << x_down.secofweek[i] << " " << x_down.x_acc[i] << " " << x_down.y_acc[i] << " " << x_down.z_acc[i] << " " << x_down.x_gyo[i] << " " << x_down.y_gyo[i] << " " << x_down.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < y_up.secofweek.size(); i++)
		{
			yup << y_up.week[i] << " " << fixed << setprecision(8) << y_up.secofweek[i] << " " << y_up.x_acc[i] << " " << y_up.y_acc[i] << " " << y_up.z_acc[i] << " " << y_up.x_gyo[i] << " " << y_up.y_gyo[i] << " " << y_up.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < y_down.secofweek.size(); i++)
		{
			ydown << y_down.week[i] << " " << fixed << setprecision(8) << y_down.secofweek[i] << " " << y_down.x_acc[i] << " " << y_down.y_acc[i] << " " << y_down.z_acc[i] << " " << y_down.x_gyo[i] << " " << y_down.y_gyo[i] << " " << y_down.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < z_up.secofweek.size(); i++)
		{
			zup << z_up.week[i] << " " << fixed << setprecision(8) << z_up.secofweek[i] << " " << z_up.x_acc[i] << " " << z_up.y_acc[i] << " " << z_up.z_acc[i] << " " << z_up.x_gyo[i] << " " << z_up.y_gyo[i] << " " << z_up.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < z_down.secofweek.size(); i++)
		{
			zdown << z_down.week[i] << " " << fixed << setprecision(8) << z_down.secofweek[i] << " " << z_down.x_acc[i] << " " << z_down.y_acc[i] << " " << z_down.z_acc[i] << " " << z_down.x_gyo[i] << " " << z_down.y_gyo[i] << " " << z_down.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < x_fwd.secofweek.size(); i++)
		{
			xfwd << x_fwd.week[i] << " " << fixed << setprecision(8) << x_fwd.secofweek[i] << " " << x_fwd.x_acc[i] << " " << x_fwd.y_acc[i] << " " << x_fwd.z_acc[i] << " " << x_fwd.x_gyo[i] << " " << x_fwd.y_gyo[i] << " " << x_fwd.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < x_rev.secofweek.size(); i++)
		{
			xrev << x_rev.week[i] << " " << fixed << setprecision(8) << x_rev.secofweek[i] << " " << x_rev.x_acc[i] << " " << x_rev.y_acc[i] << " " << x_rev.z_acc[i] << " " << x_rev.x_gyo[i] << " " << x_rev.y_gyo[i] << " " << x_rev.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < y_fwd.secofweek.size(); i++)
		{
			yfwd << y_fwd.week[i] << " " << fixed << setprecision(8) << y_fwd.secofweek[i] << " " << y_fwd.x_acc[i] << " " << y_fwd.y_acc[i] << " " << y_fwd.z_acc[i] << " " << y_fwd.x_gyo[i] << " " << y_fwd.y_gyo[i] << " " << y_fwd.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < y_rev.secofweek.size(); i++)
		{
			yrev << y_rev.week[i] << " " << fixed << setprecision(8) << y_rev.secofweek[i] << " " << y_rev.x_acc[i] << " " << y_rev.y_acc[i] << " " << y_rev.z_acc[i] << " " << y_rev.x_gyo[i] << " " << y_rev.y_gyo[i] << " " << y_rev.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < z_fwd.secofweek.size(); i++)
		{
			zfwd << z_fwd.week[i] << " " << fixed << setprecision(8) << z_fwd.secofweek[i] << " " << z_fwd.x_acc[i] << " " << z_fwd.y_acc[i] << " " << z_fwd.z_acc[i] << " " << z_fwd.x_gyo[i] << " " << z_fwd.y_gyo[i] << " " << z_fwd.z_gyo[i] << endl;
		}
		for (size_t i = 0; i < z_rev.secofweek.size(); i++)
		{
			zrev << z_rev.week[i] << " " << fixed << setprecision(8) << z_rev.secofweek[i] << " " << z_rev.x_acc[i] << " " << z_rev.y_acc[i] << " " << z_rev.z_acc[i] << " " << z_rev.x_gyo[i] << " " << z_rev.y_gyo[i] << " " << z_rev.z_gyo[i] << endl;
		}
		cout << "数据保存完成。" << endl;
		xup.close();
		xdown.close();
		yup.close();
		ydown.close();
		zup.close();
		zdown.close();
		xfwd.close();
		xrev.close();
		yfwd.close();
		yrev.close();
		zfwd.close();
		zrev.close();*/
	}
	else if (CalMode == 1)
	{
		Read_IMU_File("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\Align_30min.ASC");
	} 
	else if (CalMode == 2)
	{
		int r = 1;
		if(r==0)
		{
			FILE* FResult;
			ofstream refpos("refpos.txt");
			IMUResult result;
			int lenR_result;
			unsigned char buff_result[80];
			errno_t resulterr = fopen_s(&FResult, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\01 示例数据\\PureINS.bin", "rb");
			if (resulterr != 0 || FResult == NULL)
			{
				printf("Cannot open Result file. \n");
				return 0;
			}
			while (!feof(FResult))
			{
				if ((lenR_result = fread(buff_result, sizeof(unsigned char), 80, FResult)) < 80) break;
				DecodeResultDat(buff_result, &result);
				refpos << fixed << 2204 << "  " << setprecision(3) << result.gpsTime.SecOfWeek << setprecision(10) << "    " << result.geo.latitude << "    " << result.geo.longitude << "    " << result.geo.height << "    "
					<< result.velocity[0] << "    " << result.velocity[1] << "    " << result.velocity[2] << "    " << result.attitude.roll << "    " << result.attitude.pitch << "    " << result.attitude.yaw<< endl;
			}
			refpos.close();
		}

		FILE* FIMU;
		IMU_POS imu_pos;
		GEOCOOR geo;
		EulerAngle ea;
		XYZ xyz;
		int lenR, iter = 0;
		ofstream cal_pos("Cal_pos.txt");
		unsigned char buff[56];
		errno_t imuerr = fopen_s(&FIMU, "D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\01 示例数据\\IMU.bin", "rb");
		if (imuerr != 0 || FIMU == NULL)
		{
			printf("Cannot open IMU file. \n");
			return 0;
		}
		STATIC_IMU_DATA imu_data0, imu_data1;
		while (!feof(FIMU))
		{
			if(iter++!=0)
			{
				memcpy(&imu_data1, &imu_data0, sizeof(STATIC_IMU_DATA));
			}
			else
			{
				geo.latitude = 23.1373950708 * PI / 180.0;
				geo.longitude = 113.3713651222 * PI / 180.0;
				geo.height = 2.175;
				ea.roll = 0.0107951084511778 * PI / 180;
				ea.pitch = -2.14251290749072 * PI / 180;
				ea.yaw = -75.7498049314083 * PI / 180;
				EulerToQua(ea, imu_pos.q_1);
				for (int i = 0; i < 3; i++) 
				{
					imu_pos.geo_1.blh[i]=imu_pos.geo_2.blh[i] = geo.blh[i];
				}
			}
			if ((lenR = fread(buff, sizeof(unsigned char), 56, FIMU)) < 56) return 0;
			DecodeIMUDat(buff, &imu_data0);
			if (imu_data0.secofweek >= 91620.004)
			{
				IMUNav(&imu_data0, &imu_data1, &imu_pos, 0.005, false);
				cal_pos << fixed << 2204 << "  " << setprecision(3) << imu_data0.secofweek << setprecision(10) << "    " 
					<< imu_pos.geo_0.latitude * 180 / PI << "    " << imu_pos.geo_0.longitude * 180 / PI << "    " << imu_pos.geo_0.height <<"    "
					<< imu_pos.v_0[0] << "    " << imu_pos.v_0[1] << "    " << imu_pos.v_0[2] << "    " 
					<< QuaToEuler(imu_pos.q_0).roll * 180 / PI << "    " << QuaToEuler(imu_pos.q_0).pitch * 180 / PI << "    " << QuaToEuler(imu_pos.q_0).yaw * 180 / PI << endl;
			}
		}	
	}
	else if(CalMode == 3)
	{
		int r = 1;
		if (r == 0)ReadTRUE("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\IEproject4.ref");
		
		ifstream imufile;
		ofstream outfile;
		IMU_POS imu_pos;
		GPSTIME imu_time;
		bool is_Align = false;
		double M[3] = { 0.0 };    //初始姿态角
		vector<double>x_acc, y_acc, z_acc, x_gyo, y_gyo, z_gyo;
		vector<STATIC_IMU_DATA>imudatavec;
		STATIC_IMU_DATA imu_data0, imu_data1;
		imufile.open("D:\\HuaweiMoveData\\Users\\HUAWEI\\Desktop\\group4_rover.ASC");
		outfile.open("IMU_Nav_Position_NEW_noAve.txt");
		if (!imufile.is_open()) {
			std::cerr << "错误：无法打开文件！" << std::endl;
			return -1;
		}
		string line;
		int index = 0;
		while(getline(imufile, line))
		{
			if (line.empty()) {
				continue;
			}
			IMU_Data(line, &imu_data0);
			if (imudatavec.size() >= 10)imudatavec.erase(imudatavec.begin());
			if (imudatavec.size() < 10)imudatavec.push_back(imu_data0);
			bool is_Zero = ZeroVelocityDetect(&imu_pos.geo_0, &imu_data0);

			if (!is_Align)
			{
				if (++index == 1)imu_time = imu_data0.gpsTime;
				x_acc.push_back(imu_data0.x_acc);
				y_acc.push_back(imu_data0.y_acc);
				z_acc.push_back(imu_data0.z_acc);
				x_gyo.push_back(imu_data0.x_gyo);
				y_gyo.push_back(imu_data0.y_gyo);
				z_gyo.push_back(imu_data0.z_gyo);
				if ((imu_data0.gpsTime.Week - imu_time.Week) * 604800 + imu_data0.gpsTime.SecOfWeek - imu_time.SecOfWeek >= 300.0)
				{
					imu_data0.x_acc = mean(x_acc);
					imu_data0.y_acc = mean(y_acc);
					imu_data0.z_acc = mean(z_acc);
					imu_data0.x_gyo = mean(x_gyo);
					imu_data0.y_gyo = mean(y_gyo);
					imu_data0.z_gyo = mean(z_gyo);
					StaticAlignment(&imu_data0, M);
					cout << "静态粗对准结果：" << endl;
					cout << "横滚角roll:" << fixed << setprecision(6) << M[0] * 180 / PI << endl << "俯仰角pitch:" << M[1] * 180 / PI << endl << "航向角yaw:" << M[2] * 180 / PI << endl;
					is_Align = true;
					M[0] = -0.078084863398340 * PI / 180.0, M[1] = 0.125305964927434 * PI / 180.0, M[2] = 89.292609162662690 * PI / 180.0;
					IMUInit(M, &imu_pos);
					memcpy(&imu_data1, &imu_data0, sizeof(STATIC_IMU_DATA));
					continue;
				}
			}
			if(is_Align)
			{
				//double meanxa = 0.0, meanya = 0.0, meanza = 0.0;
				//double meanxg = 0.0, meanyg = 0.0, meanzg = 0.0;
				//for(int i=0;i<imudatavec.size();i++)
				//{
				//	meanxg += imudatavec[i].x_gyo / imudatavec.size();
				//	meanyg += imudatavec[i].y_gyo / imudatavec.size();
				//	meanzg += imudatavec[i].z_gyo / imudatavec.size();
				//}
				//imu_data0.x_gyo = meanxg;
				//imu_data0.y_gyo = meanyg;
				//imu_data0.z_gyo = meanzg;
				IMUNav(&imu_data0, &imu_data1, &imu_pos, 0.01, is_Zero);
				memcpy(&imu_data1, &imu_data0, sizeof(STATIC_IMU_DATA));
				outfile << fixed << imu_data0.week << "  " << setprecision(3) << imu_data0.secofweek << setprecision(10) << "    "
					<< imu_pos.geo_0.latitude * 180 / PI << "    " << imu_pos.geo_0.longitude * 180 / PI << "    " << imu_pos.geo_0.height << "    "
					<< imu_pos.v_0[0] << "    " << imu_pos.v_0[1] << "    " << imu_pos.v_0[2] << "    "
					<< QuaToEuler(imu_pos.q_0).roll * 180 / PI << "    " << QuaToEuler(imu_pos.q_0).pitch * 180 / PI << "    " << QuaToEuler(imu_pos.q_0).yaw * 180 / PI << endl;
			}
		}
		imufile.close();
		outfile.close();
	}

	return 0;
}