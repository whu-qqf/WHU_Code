/**
* @file     IMU.h
* @brief    这个文件给该IMU项目相关函数声明
* @details  mean				均值计算函数\n
*           RM					子午圈曲率半径计算函数\n
* 		    RN					卯酉圈曲率半径计算函数\n
* 		    Omega_ie_n			n系地球自转角速度计算函数\n
*		    G_p_n				n系重力加速度计算函数\n
* 		    Omega_en_n			n系位移角速度计算函数\n
*           Integral			积分函数\n
* 		    Read_IMU_File		读取IMU原始数据文件函数\n
*           ACC_Cal				加速度计标定函数\n
* 		    GYR_Cal				陀螺仪标定函数\n
* 		    GYR_Bias			利用加速度计六位置法静态数据标定陀螺零偏函数\n
* 		    Compensation		误差补偿函数\n
* 		    StaticAlignment		静态粗对准函数\n
* 		    AllanVariance		Allan方差计算函数\n
*           IMU_Data			解析IMU数据行函数\n
* 			QuaToEuler			四元数转欧拉角函数\n
* 			EulerToQua			欧拉角转四元数函数\n
* 			EulerToDCM          欧拉角转方向余弦矩阵函数\n
* 			QuaNorm             四元数归一化函数\n
* 			EquRotToQua         等效旋转向量转四元数函数\n
* 			QuaToDCM            四元数转方向余弦矩阵函数\n
* 			ReadTRUE            读取参考真值文件函数\n
* 			IMUInit             惯导初始化函数\n
* 			ZeroVelocityDetect  零速度检测函数\n
* 			VelocityZUPT        零速修正函数\n
* 			VelocityUpdate      机械编排速度更新函数\n
* 			PositionUpdate      机修编排位置更新函数\n
* 			AttitudeUpdate      机械编排姿态更新函数\n
* 			IMUNav              惯导导航解算函数
* @author   Qifeng Qin. Email: 2023302143029@whu.edu.cn
* @date     2025/11/29
* @version  0.0.01
* @par      Copyright(c) 2012-2021 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
* @par      History:
*           2025/11/29,Qifeng Qin, Modify and finish.
*/

#include<iostream>
#include<sstream>
#include<fstream>
#include<vector>
#include<string>
#include<algorithm>
#include<iomanip>
#include<cmath>
#include"IMU.h"
#include"const.h"
#include"Matrix.h"
#include"struct.h"

static const double omega_ie_e[3] = { 0.0, 0.0, OMEGA_E };

/**
* @brief       均值计算
* @param[in]   data  const vector<T>  待求均值数据
* @return      double                 数据均值
* @note        使用前需要明确vector数据类型
*/
template <typename T>
double mean(const vector<T>data)
{
    double sum = 0.0;
    for (int i = 0; i < size(data); i++)sum += data[i] * 1.0;
    return sum / size(data);
}


/**
* @brief       子午圈曲率半径计算
* @param[in]   lat  const double  纬度[rad]
* @return      double             子午圈半径[m]
* @note        nothing.
*/
double RM(const double lat)
{
    return R_WGS84 * (1 - e2_WGS84) / sqrt(pow(1 - e2_WGS84  * sin(lat) * sin(lat), 3));
}


/**
* @brief       卯酉圈曲率半径计算
* @param[in]   lat  const double  纬度[rad]
* @return      double             卯酉圈半径[m]
* @note        nothing.
*/
double RN(const double lat)
{
    return R_WGS84 / sqrt(1 -  e2_WGS84 * sin(lat) * sin(lat));
}


/**
* @brief       n系地球自转角速度计算
* @param[in]   lat       const double   纬度[rad]
* @param[out]  omega_ie  double[]       n系地球自转角速度[rad/s,rad/s,rad/s]
* @return      void
* @note        nothing.
*/
void Omega_ie_n(const double lat, double omega_ie[])
{
    omega_ie[0] = OMEGA_E * cos(lat), omega_ie[1] = 0.0, omega_ie[2] = -OMEGA_E * sin(lat);
}


/**
* @brief       n系重力加速度计算
* @param[in]   geo  const GEOCOOR*   大地坐标系坐标[rad,rad,m]
* @param[out]  g_p  double[]         n系重力加速度[m/s²,m/s²,m/s²]
* @return      void
* @note        有两种算法，一种是精密公式，另一种是二阶展开式
*/
void G_p_n(const GEOCOOR* geo, double g_p[])
{
    //double m = OMEGA_E * OMEGA_E * R_WGS84 * R_WGS84 * B_WGS84 / GM_GPS;
	//double g_phi = (R_WGS84 * gama_a * cos(geo->latitude) * cos(geo->latitude) + B_WGS84 * gama_b * sin(geo->latitude) * sin(geo->latitude)) / sqrt(R_WGS84 * R_WGS84 * cos(geo->latitude) * cos(geo->latitude) + B_WGS84 * B_WGS84 * sin(geo->latitude) * sin(geo->latitude));
    //double g_h_phi = g_phi * (1 - 2 * (1 + F_WGS84 + m - 2 * F_WGS84 * sin(geo->latitude) * sin(geo->latitude)) * geo->height / R_WGS84 + 3 * geo->height * geo->height / R_WGS84 / R_WGS84);
	//g_p[0] = 0.0, g_p[1] = 0.0, g_p[2] = g_h_phi;
	double g_0 = 9.7803267715 * (1 + 0.0052790414 * sin(geo->latitude) * sin(geo->latitude) + 0.0000232718 * sin(geo->latitude) * sin(geo->latitude) * sin(geo->latitude) * sin(geo->latitude));
	double g_h_phi = g_0 - (3.087691089E-6 - 4.397731E-9 * sin(geo->latitude) * sin(geo->latitude)) * geo->height + 0.721E-12 * geo->height * geo->height;
	g_p[0] = 0.0, g_p[1] = 0.0, g_p[2] = g_h_phi;
}


/**
* @brief       n系位移角速度计算
* @param[in]   geo  const GEOCOOR*   大地坐标系坐标[rad,rad,m]
* @param[in]   v    const double[]   n系速度NED[m/s,m/s,m/s]
* @param[out]  w    double[]         n系位移角速度[rad/s,rad/s,rad/s]
* @return      void
* @note        注意v是NED速度
*/
void Omega_en_n(const GEOCOOR* geo, const double v[], double w[])
{
    w[0] = v[1] / (RN(geo->latitude) + geo->height);
    w[1] = -v[0] / (RM(geo->latitude) + geo->height);
	w[2] = -v[1] * tan(geo->latitude) / (RN(geo->latitude) + geo->height);
}


/**
* @brief       积分
* @param[in]   data    const vector<double>   数据
* @param[in]   time    const vector<double>   数据时间
* @return      double                         数据积分结果
* @note        保证data和time长度一致
*/
double Integral(const vector<double>data, const vector<double>time)
{
    double sum = 0.0;
    for (int i = 0; i < size(data)-1; i++)
    {
        sum += (data[i + 1] + data[i]) * (time[i + 1] - time[i]) / 2.0;
    }
    return sum;
}


/**
* @brief       求和（内部调用函数）
* @param[in]   data    const vector<double>   数据
* @return      double                         数据积分结果
* @note        nothing.
*/
double Add(const vector<double>data)
{
    double sum = 0.0;
    for (int i = 0; i < size(data) - 1; i++)
    {
        sum += data[i];
    }
    return sum;
}


/**
* @brief       读取IMU原始数据文件(整个文件保存)
* @param[in]   filename    const string  IMU数据文件路径
* @param[out]  imu_data    IMU_DATA*     IMU数据结构体
* @return      int         0表示读取成功；-1表示失败
* @note        nothing.
*/
int Read_IMU_File(const string filename, IMU_DATA* imu_data)
{
    ifstream file(filename);
    string line;
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return -1;
    }
    while (getline(file, line))
    {
        replace(line.begin(), line.end(), ';', ',');
        replace(line.begin(), line.end(), '*', ',');
        stringstream parts(line);
        string part;
        int week,index = 0;
        double secofweek, x_a, y_a, z_a, x_g, y_g, z_g;
        while (getline(parts, part, ','))
        {
            if (index == 3)week = stoi(part);
            if (index == 4)secofweek = stod(part);
            if (index == 6)z_a = stod(part) * acc_scale * 100;
            if (index == 7)y_a = -stod(part) * acc_scale * 100;
            if (index == 8)x_a = stod(part) * acc_scale * 100;
            if (index == 9)z_g = stod(part) * gyo_scale * 100;
            if (index == 10)y_g = -stod(part) * gyo_scale * 100;
            if (index == 11)x_g = stod(part) * gyo_scale * 100;
            index++;
        }
        imu_data->week.push_back(week);
        imu_data->secofweek.push_back(secofweek);
        imu_data->x_acc.push_back(x_a);
        imu_data->y_acc.push_back(y_a);
        imu_data->z_acc.push_back(z_a);
        imu_data->x_gyo.push_back(x_g);
        imu_data->y_gyo.push_back(y_g);
        imu_data->z_gyo.push_back(z_g);
    }
    return 0;
}


/**
* @brief       加速度计标定（六位置法标定）
* @param[in]   data_set    const IMU_DATA*  IMU数据集（六位置法）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void ACC_Cal(const IMU_DATA* data_set, IMU_ERROR* imu_error)
{
    double L[3 * 6] = { mean(data_set[0].x_acc),mean(data_set[1].x_acc),mean(data_set[2].x_acc),mean(data_set[3].x_acc) ,mean(data_set[4].x_acc),mean(data_set[5].x_acc),
        mean(data_set[0].y_acc),mean(data_set[1].y_acc),mean(data_set[2].y_acc),mean(data_set[3].y_acc) ,mean(data_set[4].y_acc),mean(data_set[5].y_acc),
        mean(data_set[0].z_acc),mean(data_set[1].z_acc),mean(data_set[2].z_acc),mean(data_set[3].z_acc) ,mean(data_set[4].z_acc),mean(data_set[5].z_acc)
    };
    double A[4 * 6] = { g,-g,0,0,0,0,
        0,0,g,-g,0,0,
        0,0,0,0,g,-g,
        1,1,1,1,1,1
    };
    double AT[6 * 4] = { 0.0 }, AAT[4 * 4] = { 0.0 }, INAAT[4 * 4] = { 0.0 }, LAT[3 * 4] = { 0.0 };
    MatrixTranspose(4, 6, A, AT);
    MatrixMultiply(3, 6, 6, 4, L, AT, LAT);
    MatrixMultiply(4, 6, 6, 4, A, AT, AAT);
    MatrixInverse(4, 4, AAT, INAAT);
    MatrixMultiply(3, 4, 4, 4, LAT, INAAT, imu_error->M);
}


/**
* @brief       陀螺仪计标定
* @param[in]   fwd_data    const IMU_DATA*  IMU前向旋转数据
* @param[in]   rev_data    const IMU_DATA*  IMU后向旋转数据
* @param[in]   index       const int        轴索引（0：x轴；1：y轴；2：z轴）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void GYR_Cal(const IMU_DATA* fwd_data, const IMU_DATA* rev_data, const int index, IMU_ERROR* imu_error)
{
    double t_fwd = fwd_data->secofweek[0], t_rev = rev_data->secofweek[0];
    vector<double>fwd_gyo = *(fwd_data->GYO_DATA[index]);
    vector<double>rev_gyo = *(rev_data->GYO_DATA[index]);
    int fwd_index_begin = 0, fwd_index_end = 0;
    int rev_index_begin = 0, rev_index_end = 0;
    for (int i = 0; i < size(fwd_gyo) - 4; i++)
    {
        if ((fwd_gyo[i + 4] - fwd_gyo[i]) * 180 / PI > 0.1)
        {
            fwd_index_begin = i;
            break;
        }
    }
    for (int i = size(fwd_gyo) - 1; i >= 4; i--)
    {
        if ((fwd_gyo[i - 4] - fwd_gyo[i])* 180 / PI > 0.1)
        {
            fwd_index_end = i;
            break;
        }
    }
    for (int i = 0; i < size(rev_gyo) - 4; i++)
    {
        if ((rev_gyo[i + 4] - rev_gyo[i]) * 180 / PI < -0.1)
        {
            rev_index_begin = i;
            break;
        }
    }
    for (int i = size(rev_gyo) - 1; i >= 4; i--)
    {
        if ((rev_gyo[i - 4] - rev_gyo[i]) * 180 / PI < -0.1)
        {
            rev_index_end = i;
            break;
        }
    } 
    if (fwd_index_end - fwd_index_begin > rev_index_end - rev_index_begin)
    {
        do {
            rev_index_begin--;
        } while (fwd_index_end - fwd_index_begin != rev_index_end - rev_index_begin);
    }
    else if (fwd_index_end - fwd_index_begin < rev_index_end - rev_index_begin)
    {
        do {
            fwd_index_begin--;
        } while (fwd_index_end - fwd_index_begin != rev_index_end - rev_index_begin);
    }
    double dt = (fwd_data->secofweek[fwd_index_end] - fwd_data->secofweek[fwd_index_begin] + rev_data->secofweek[rev_index_end] - rev_data->secofweek[rev_index_begin]) / 2.0;
    vector<double> subdata_fwd(fwd_gyo.begin() + fwd_index_begin, fwd_gyo.begin() + fwd_index_end + 1);
    vector<double> subdata_rev(rev_gyo.begin() + rev_index_begin, rev_gyo.begin() + rev_index_end + 1);
    double angle_fwd = Add(subdata_fwd)/100;
    double angle_rev = Add(subdata_rev)/100;
    double bias = (angle_fwd + angle_rev) / (2 * dt) - (OMEGA_E) * sin(LOCAL_LAT * PI / 180);
    double scaling = (angle_fwd - angle_rev) / (4*PI) - 1;
    imu_error->Gyo[4 * index + 3] = bias ;
    imu_error->Gyo[4 * index + index] = 1+scaling;
}


/**
* @brief       利用加速度计六位置法静态数据标定陀螺零偏
* @param[in]   imu_data    const IMU_DATA*  IMU数据结构体数组（六位置法）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void GYR_Bias(const IMU_DATA* imu_data, IMU_ERROR* imu_error)
{
    imu_error->Gyo[3] = (mean(imu_data[0].x_gyo) + mean(imu_data[1].x_gyo)) / 2;
    imu_error->Gyo[7] = (mean(imu_data[2].y_gyo) + mean(imu_data[3].y_gyo))  / 2;
    imu_error->Gyo[11] =(mean(imu_data[4].z_gyo) + mean(imu_data[5].z_gyo))  / 2;
    return;
}


/**
* @brief       误差补偿
* @param[in]   imu_error   const IMU_ERROR*  IMU误差结构体
* @param[out]  imu_data    IMU_DATA*         待补偿的IMU数据结构体
* @return      void
* @note        nothing.
*/
void Compensation(IMU_DATA* imu_data, const IMU_ERROR* imu_error)
{
    double Z[3 * 1] = { 0.0 }, b[3 * 1] = { 0.0 }, Zb[3 * 1] = { 0.0 }, X[3 * 1] = { 0.0 };
    double H_acc[3 * 3] = { imu_error->M[0],imu_error->M[1] ,imu_error->M[2] ,
        imu_error->M[4] ,imu_error->M[5] ,imu_error->M[6] ,
        imu_error->M[8] ,imu_error->M[9] ,imu_error->M[10] };
    double H_gyo[3 * 3] = { imu_error->Gyo[0],imu_error->Gyo[1] ,imu_error->Gyo[2] ,
        imu_error->Gyo[4],imu_error->Gyo[5] ,imu_error->Gyo[6] ,
        imu_error->Gyo[8],imu_error->Gyo[9] ,imu_error->Gyo[10] };
    double INH_acc[3 * 3] = { 0.0 }, INH_gyo[3 * 3] = { 0.0 };
    MatrixInverse(3, H_acc, INH_acc);
    MatrixInverse(3, H_gyo, INH_gyo);
    for (int i = 0; i < size(imu_data->secofweek); i++)
    {
		Z[0] = imu_data->x_acc[i];
		Z[1] = imu_data->y_acc[i];
		Z[2] = imu_data->z_acc[i];
		b[0] = imu_error->M[3];
		b[1] = imu_error->M[7];
		b[2] = imu_error->M[11];
		MatrixSub(3, 1, Z, b, Zb);
		MatrixMultiply(3, 3, 3, 1, INH_acc, Zb, X);
		imu_data->x_acc[i] = X[0];
		imu_data->y_acc[i] = X[1];
		imu_data->z_acc[i] = X[2];
		Z[0] = imu_data->x_gyo[i];
		Z[1] = imu_data->y_gyo[i];
		Z[2] = imu_data->z_gyo[i];
		b[0] = imu_error->Gyo[3];
		b[1] = imu_error->Gyo[7];
		b[2] = imu_error->Gyo[11];
		MatrixSub(3, 1, Z, b, Zb);
		MatrixMultiply(3, 3, 3, 1, INH_gyo, Zb, X);
		imu_data->x_gyo[i] = X[0];
		imu_data->y_gyo[i] = X[1];
		imu_data->z_gyo[i] = X[2];
    }
}


/**
* @brief       Allan方差计算
* @param[in]   data        const vector<double>  待计算数据
* @param[in]   tau0        const double          采样时间间隔
* @param[in]   filename    const string          输出文件名
* @return      void
* @note        nothing.
*/
void AllanVariance(const vector<double>data, const double tau0, const string filename)
{
    int N = data.size();
    if (N < 2)return;
    vector<double>prefix(N + 1, 0.0);
    ofstream outfile;
    outfile.open(filename);

    for (int i = 0; i < N; i++)prefix[i + 1] = prefix[i] + data[i];
    int max_n = N / 2;
    for (int n = 1; n <= max_n; n++)
    {
        int M = N / n;
        if (M < 2)continue;
        vector<double>y(M);
        for (int k = 0; k < M; k++)
        {
            int start = k * n;
            int end = start + n;
            y[k] = (prefix[end] - prefix[start]) / n;
        }
        double sum = 0.0;
        for (int k = 0; k < M - 1; k++) 
        {
            double diff = y[k + 1] - y[k];
            sum += diff * diff;
        }
        double sigma2 = sum / (2 * (M - 1));
        outfile << n * tau0 << " " << fixed << setprecision(8) << sqrt(sigma2) << endl;
    }
}


/**
* @brief       静态粗对准
* @param[in]   static_imu_data    const STATIC_IMU_DATA*  静态IMU数据
* @param[out]  M                  double*                 初始姿态角（roll,pitch,yaw）[rad,rad,rad]
* @return      void
* @note        nothing.
*/
void StaticAlignment(const STATIC_IMU_DATA* static_imu_data, double* M)
{
    double gn[3] = { 0,0,g }, omegan[3] = { OMEGA_E * cos(CAR_LAT * PI / 180),0,-OMEGA_E * sin(CAR_LAT * PI / 180) };
    double gb[3] = { -static_imu_data->x_acc,-static_imu_data->y_acc,-static_imu_data->z_acc };
    double wb[3] = { static_imu_data->x_gyo,static_imu_data->y_gyo,static_imu_data->z_gyo };
    double vg[3] = { 0.0 }, vw[3] = { 0.0 }, vgw[3] = { 0.0 };
    double wg[3] = { 0.0 }, ww[3] = { 0.0 }, wgw[3] = { 0.0 };
    Mat3xMat3(gn, omegan, vw);
    Mat3xMat3(vw, gn, vgw);		
    Mat3xMat3(gb, wb, ww);
    Mat3xMat3(ww, gb, wgw);
    double mvw = modMat(vw),mvgw = modMat(vgw),mww = modMat(ww),mwgw = modMat(wgw);
    for (int i = 0; i < 3; i++)
    {
        vg[i] = gn[i] / modMat(gn);
        vw[i] = vw[i] / mvw;
        vgw[i] = vgw[i] / mvgw;
        wg[i] = gb[i] / modMat(gb);
        ww[i] = ww[i] / mww;
        wgw[i] = wgw[i] / mwgw;
    }
    double M1[9] = { vg[0],vg[1],vg[2],vw[0],vw[1],vw[2],vgw[0],vgw[1],vgw[2] };
    double M2[9] = { wg[0],wg[1],wg[2],ww[0],ww[1],ww[2],wgw[0],wgw[1],wgw[2] };
    double INM1[9] = { 0.0 }, Cbn[9] = { 0.0 };
    MatrixInverse(3, M1, INM1);
    MatrixMultiply(3, 3, 3, 3, INM1, M2, Cbn);
    double theta = atan(-Cbn[6] / sqrt(Cbn[7] * Cbn[7] + Cbn[8] * Cbn[8]));
    double phi = atan2(Cbn[7], Cbn[8]);
	double psi = atan2(Cbn[3], Cbn[0]);
	M[0] = phi;     //roll
	M[1] = theta;  //pitch
	M[2] = psi;   //yaw
}


/**
* @brief       IMU数据读取函数（@静态粗对准入口）
* @param[in]   filename  const string     IMU数据文件路径
* @return      int       0表示运行成功；-1表示失败
* @note        nothing.
*/
int Read_IMU_File(const string filename)
{
    ifstream file(filename);
    STATIC_IMU_DATA static_imu_data;
    string line;
    int CalMode = -1;
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return -1;
    }
    cout << "请选择：（0）全历元平均解算；（1）每秒平均解算；（2）每历元单独解算" << endl;
	cin >> CalMode;
	ofstream outfile;
    double M[3] = { 0.0 };
	outfile.open("Attitude.txt");
    outfile << "周内秒 " << "俯仰角pitch " << "横滚角roll " << "航向角yaw " << endl;
    vector<double>x_acc, y_acc, z_acc, x_gyo, y_gyo, z_gyo;
    double secofweek = 0.0, presec = 0.0;
    while (getline(file, line))
    {
        replace(line.begin(), line.end(), ';', ',');
        replace(line.begin(), line.end(), '*', ',');
        stringstream parts(line);
        string part;
        int index = 0;
        double x_a = 0.0, y_a = 0.0, z_a = 0.0, x_g = 0.0, y_g = 0.0, z_g = 0.0;
        while (getline(parts, part, ','))
        {
			if (index == 2)secofweek = stod(part);
            if (index == 8)z_a = stod(part) * novatel_acc_scale * 200;
            if (index == 9)y_a = -stod(part) * novatel_acc_scale * 200;
            if (index == 10)x_a = stod(part) * novatel_acc_scale * 200;
            if (index == 11)z_g = stod(part) * novatel_gyo_scale * 200;
            if (index == 12)y_g = -stod(part) * novatel_gyo_scale * 200;
            if (index == 13)x_g = stod(part) * novatel_gyo_scale * 200;
            index++;
        }
        if (int(secofweek) - int(presec) == 1 && CalMode==1)
        {
            static_imu_data.x_acc = mean(x_acc);
            static_imu_data.y_acc = mean(y_acc);
            static_imu_data.z_acc = mean(z_acc);
            static_imu_data.x_gyo = mean(x_gyo);
            static_imu_data.y_gyo = mean(y_gyo);
            static_imu_data.z_gyo = mean(z_gyo);
            StaticAlignment(&static_imu_data, M);
            outfile << int(secofweek) << " " << fixed << setprecision(8) << M[0] << " " << M[1] << " " << M[2]<<endl;
            x_acc.clear();
            y_acc.clear();
            z_acc.clear();
            x_gyo.clear();
            y_gyo.clear();
            z_gyo.clear();
        }
        if (CalMode == 2)
        {
            static_imu_data.x_acc = x_a;
            static_imu_data.y_acc = y_a;
            static_imu_data.z_acc = z_a;
            static_imu_data.x_gyo = x_g;
            static_imu_data.y_gyo = y_g;
            static_imu_data.z_gyo = z_g;
            StaticAlignment(&static_imu_data, M);
            outfile << fixed<< setprecision(3)<< secofweek << " " << setprecision(8) << M[0] << " " << M[1] << " " << M[2] << endl;
        }
        x_acc.push_back(x_a);
        y_acc.push_back(y_a);
        z_acc.push_back(z_a);
        x_gyo.push_back(x_g);
        y_gyo.push_back(y_g);
        z_gyo.push_back(z_g);
        presec = secofweek;
    }
    if (CalMode == 0)
    {
        static_imu_data.x_acc = mean(x_acc);
        static_imu_data.y_acc = mean(y_acc);
        static_imu_data.z_acc = mean(z_acc);
        static_imu_data.x_gyo = mean(x_gyo);
        static_imu_data.y_gyo = mean(y_gyo);
        static_imu_data.z_gyo = mean(z_gyo);
        AllanVariance(x_acc, 0.005, "x_acc.txt");
        AllanVariance(y_acc, 0.005, "y_acc.txt");
        AllanVariance(z_acc, 0.005, "z_acc.txt");
        AllanVariance(x_gyo, 0.005, "x_gyo.txt");
        AllanVariance(y_gyo, 0.005, "y_gyo.txt");
        AllanVariance(z_gyo, 0.005, "z_gyo.txt");

        StaticAlignment(&static_imu_data, M);
        cout << "俯仰角pitch:" << fixed << setprecision(6)<< M[0] << endl << "横滚角roll:" << M[1] << endl << "航向角yaw:" << M[2] << endl;
    }
    outfile.close();
    return 0;
}


/**
* @brief       解析IMU数据行
* @param[in]   line      const string     IMU数据行
* @param[out]  imu_data  STATIC_IMU_DATA* IMU数据结构体
* @return      void
* @note        此次使用STATIC_IMU_DATA来存储单条IMU数据，避免重复定义结构体
*/
void IMU_Data(const string line, STATIC_IMU_DATA* imu_data)
{
    string imu_line = line;
    replace(imu_line.begin(), imu_line.end(), ';', ',');
    replace(imu_line.begin(), imu_line.end(), '*', ',');
    stringstream parts(imu_line);
    string part;
    int index = 0;
    double secofweek = 0.0, x_a = 0.0, y_a = 0.0, z_a = 0.0, x_g = 0.0, y_g = 0.0, z_g = 0.0;
    while (getline(parts, part, ','))
    {
        /*if (index == 3)imu_data->week = stoi(part);
        if (index == 4)imu_data->secofweek = stod(part);
        if (index == 6)imu_data->z_acc = stod(part) * acc_scale * 100;
        if (index == 7)imu_data->y_acc = -stod(part) * acc_scale * 100;
        if (index == 8)imu_data->x_acc = stod(part) * acc_scale * 100;
        if (index == 9)imu_data->z_gyo = stod(part) * gyo_scale * 100;
        if (index == 10)imu_data->y_gyo = -stod(part) * gyo_scale * 100;
        if (index == 11)imu_data->x_gyo = stod(part) * gyo_scale * 100;
        imu_data->gpsTime.Week = imu_data->week;
        imu_data->gpsTime.SecOfWeek = imu_data->secofweek;
        index++;*/
        if (index == 5)imu_data->week = stoi(part);
		if (index == 6)imu_data->secofweek = stod(part);

        if (index == 13)imu_data->z_acc = -stod(part) * chcn_accel_scale;
        if (index == 14)imu_data->x_acc = -stod(part) * chcn_accel_scale;
        if (index == 15)imu_data->y_acc = stod(part) * chcn_accel_scale;

        if (index == 16)imu_data->z_gyo = -stod(part) * chcn_gyo_scale * PI / 180;
        if (index == 17)imu_data->x_gyo = -stod(part) * chcn_gyo_scale * PI / 180;
        if (index == 18)imu_data->y_gyo = stod(part) * chcn_gyo_scale * PI / 180;
		imu_data->gpsTime.Week = imu_data->week;
		imu_data->gpsTime.SecOfWeek = imu_data->secofweek;
        index++;
    }
}


/**
* @brief           四元数归一化
* @param[in/out]   q      double[]     四元数
* @return          void
* @note            nothing.
*/
void QuaNorm(double q[])
{
	double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if(norm<1e-15)
    {
        q[0]=1.0; q[1]=0.0; q[2]=0.0; q[3]=0.0;
        return;
	}
    for(int i=0;i<4;i++)
    {
        q[i] = q[i] / norm;
	}
}


/**
* @brief       四元数转欧拉角
* @param[in]   q   const double[]    四元数
* @return      EulerAngle            欧拉角结构体
* @note        nothing.
*/
EulerAngle QuaToEuler(const double q[])
{
    EulerAngle ea;
	ea.roll = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	ea.pitch = asin(2 * (q[0] * q[2] - q[3] * q[1]));
	ea.yaw = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
	return ea;
}


/**
* @brief       欧拉角转四元数
* @param[in]   ea  const EulerAngle  欧拉角结构体
* @param[out]  q   const double[]    四元数
* @return      void
* @note        nothing.
*/
void EulerToQua(const EulerAngle ea, double q[])
{
    double cos_r = cos(ea.roll / 2.0);
    double sin_r = sin(ea.roll / 2.0);
    double cos_p = cos(ea.pitch / 2.0);
    double sin_p = sin(ea.pitch / 2.0);
    double cos_y = cos(ea.yaw / 2.0);
    double sin_y = sin(ea.yaw / 2.0);

    q[0] = cos_y * cos_p * cos_r + sin_y * sin_p * sin_r;
    q[1] = cos_y * cos_p * sin_r - sin_y * sin_p * cos_r;
    q[2] = cos_y * sin_p * cos_r + sin_y * cos_p * sin_r;
    q[3] = sin_y * cos_p * cos_r - cos_y * sin_p * sin_r;
	QuaNorm(q);
}


/**
* @brief       欧拉角转方向余弦矩阵
* @param[in]   ea  const EulerAngle  欧拉角结构体
* @param[out]  C   double[]          方向余弦矩阵
* @return      void
* @note        nothing.
*/
void EulerToDCM(const EulerAngle ea, double C[])
{
    double phi = ea.roll;    // 横滚角x
    double theta = ea.pitch; // 俯仰角y
    double psi = ea.yaw;     // 航向角z
    double c_phi = cos(phi);
    double s_phi = sin(phi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_psi = cos(psi);
    double s_psi = sin(psi);

    C[0] = c_theta * c_psi;
    C[3] = c_theta * s_psi;
    C[6] = -s_theta;

    C[1] = -c_phi * s_psi + s_phi * s_theta * c_psi;
    C[4] = c_phi * c_psi + s_phi * s_theta * s_psi;
    C[7] = s_phi * c_theta;

    C[2] = s_phi * s_psi + c_phi * s_theta * c_psi;
    C[5] = -s_phi * c_psi + c_phi * s_theta * s_psi;
    C[8] = c_phi * c_theta;
}


/**
* @brief       四元数转方向余弦矩阵
* @param[in]   q   const double[]  四元数
* @param[out]  C   double[]        方向余弦矩阵
* @return      void
* @note        nothing.
*/
void QuaToDCM(const double q[], double C[])
{
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];
    C[0] = w * w + x * x - y * y - z * z;
    C[1] = 2 * (x * y - w * z);
    C[2] = 2 * (x * z + w * y);;
    C[3] = 2 * (x * y + w * z);;
    C[4] = w * w - x * x + y * y - z * z;
    C[5] = 2 * (y * z - w * x);
    C[6] = 2 * (x * z - w * y);
    C[7] = 2 * (y * z + w * x);
    C[8] = w * w - x * x - y * y + z * z;
}


/**
* @brief       等效旋转矢量转四元数
* @param[in]   equRot  const double[]  等效旋转矢量
* @param[out]  q       double[]        四元数
* @return      void
* @note        nothing.
*/
void EquRotToQua(const double equRot[], double q[])
{
    double equRot_half[3] = { 0.0 };
    for (int i = 0; i < 3; i++)equRot_half[i] = 0.5 * equRot[i];
    double mod_half = modMat(equRot_half);
    q[0] = cos(mod_half);
    if (mod_half < 1e-15) {   // 零旋转
        q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
        return;
    }
    for (int i = 1; i < 4; i++)q[i] = sin(mod_half) * 0.5 * equRot[i - 1] / mod_half;
    QuaNorm(q);
}


/**
* @brief       读取参考真值文件
* @param[in]   filename  const string     参考真值文件路径
* @return      void
* @note        nothing.
*/
void ReadTRUE(const string filename)
{
    ifstream file(filename);
	ofstream outfile("TrueIMU.txt");
    string line;
    while (getline(file, line))
    {
        if (line.empty()) continue;
		short week = stoi(line.substr(0, 4));
		double secofweek = stod(line.substr(5, 11));
		double lat = stod(line.substr(17, 14));
		double lon = stod(line.substr(30, 15));
		double height = stod(line.substr(52, 6));
		double vE = stod(line.substr(214, 8));
		double vN = stod(line.substr(223, 8));
		double vD = -stod(line.substr(232, 9));
        double yaw = stod(line.substr(241, 15));
		double pitch = stod(line.substr(257, 14));
		double roll = stod(line.substr(270, 15));
        outfile << fixed << week << "  " << setprecision(3) << secofweek << setprecision(10) << "    "
            << lat << "    " << lon << "    " << height << "    "
            << vN << "    " << vE << "    " << vD << "    "
            << roll<< "    " <<pitch  << "    " <<yaw << endl;
    }
    file.close();
	outfile.close();
}


/**
* @brief       IMU初始化
* @param[in]   M          const double[]   初始姿态角（roll,pitch,yaw）[rad,rad,rad]
* @param[out]  imu_pos    IMU_POS*         IMU位置姿态
* @return      void
* @note        nothing.
*/
void IMUInit(const double M[], IMU_POS* imu_pos)
{
    imu_pos->geo_0.latitude=imu_pos->geo_1.latitude = imu_pos->geo_2.latitude = 30.527970844292670 * PI / 180.0;
    imu_pos->geo_0.longitude=imu_pos->geo_1.longitude = imu_pos->geo_2.longitude = 114.3556147193078 * PI / 180.0;
    imu_pos->geo_0.height=imu_pos->geo_1.height = imu_pos->geo_2.height = 19.687915471697682;
    EulerAngle ea;
    ea.roll = M[0];
    ea.pitch = M[1];
    ea.yaw = M[2];
    EulerToQua(ea, imu_pos->q_1);
    memcpy(imu_pos->q_0, imu_pos->q_1, 4 * sizeof(double));

}


/**
* @brief       零速检测
* @param[in]   geo        const GEOCOOR*           大地坐标系坐标[rad,rad,m]
* @param[in]   imu_data   const STATIC_IMU_DATA*   单历元IMU数据
* @return      bool		  是否为零速，true表示是零速, false表示非零速
* @note        nothing.
*/
bool ZeroVelocityDetect(const GEOCOOR* geo, const STATIC_IMU_DATA* imu_data)
{
    //角速度模长
    double gyro_norm = sqrt(imu_data->x_gyo * imu_data->x_gyo +
        imu_data->y_gyo * imu_data->y_gyo +
        imu_data->z_gyo * imu_data->z_gyo);
    //加速度模长
    double accel_norm = sqrt(imu_data->x_acc * imu_data->x_acc +
        imu_data->y_acc * imu_data->y_acc +
        imu_data->z_acc * imu_data->z_acc);
    //计算正常重力加速度
    double m = OMEGA_E * OMEGA_E * R_WGS84 * R_WGS84 * B_WGS84 / GM_GPS;
    double g_phi = (R_WGS84 * gama_a * cos(geo->latitude) * cos(geo->latitude) + B_WGS84 * gama_b * sin(geo->latitude) * sin(geo->latitude)) / sqrt(R_WGS84 * R_WGS84 * cos(geo->latitude) * cos(geo->latitude) + B_WGS84 * B_WGS84 * sin(geo->latitude) * sin(geo->latitude));
    double g_h_phi = g_phi * (1 - 2 * (1 + F_WGS84 + m - 2 * F_WGS84 * sin(geo->latitude) * sin(geo->latitude)) * geo->height / R_WGS84 + 3 * geo->height * geo->height / R_WGS84 / R_WGS84);
    return abs(accel_norm - g_h_phi) < 0.05 && abs(gyro_norm) < 0.0006;
}


/**
* @brief          零速修正
* @param[in/out]  imu_pos   IMU_POS*    解算得到的IMU状态结构体
* @return         void
* @note           nothing.
*/
void VelocityZUPT(IMU_POS* imu_pos)
{
    for (int i = 0; i < 3; i++) imu_pos->v_0[i] = 0.0;
}


/**
* @brief       速度修正
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void VelocityUpdate(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt)
{
	GEOCOOR pos_half_blh;
    //外推中间时刻位置和速度
	double v_half[3] = { 0.0 };
    for(int i=0;i<3;i++)    
    {
		pos_half_blh.blh[i] = 1.5 * imu_pos->geo_1.blh[i] - 0.5 * imu_pos->geo_2.blh[i];
        v_half[i] = 1.5 * imu_pos->v_1[i] - 0.5 * imu_pos->v_2[i];
	}
    //用中间时刻位置和速度重新计算地理参数
    double Rm = RM(pos_half_blh.latitude), Rn = RN(pos_half_blh.latitude);
    double omega_ie_n[3] = { 0.0 }, omega_en_n[3] = { 0.0 }, g_p_n[3] = { 0.0 };
    Omega_ie_n(pos_half_blh.latitude, omega_ie_n);
	G_p_n(&pos_half_blh, g_p_n);
	Omega_en_n(&pos_half_blh, v_half, omega_en_n);
    //计算 b 系比力积分项(补偿旋转和划桨效应)，并投影到n系
    double v0[3] = { imu_data0->x_acc * dt,imu_data0->y_acc * dt,imu_data0->z_acc * dt };
    double v1[3] = { imu_data1->x_acc * dt,imu_data1->y_acc * dt,imu_data1->z_acc * dt };
    double sita0[3] = { imu_data0->x_gyo * dt,imu_data0->y_gyo * dt,imu_data0->z_gyo * dt };
    double sita1[3] = { imu_data1->x_gyo * dt,imu_data1->y_gyo * dt,imu_data1->z_gyo * dt };
    double v_fk_b[3] = { 0.0 }, sita0v0[3] = { 0.0 }, sita1v0[3] = { 0.0 }, v1sita0[3] = { 0.0 };
	Mat3xMat3(sita0, v0, sita0v0);
	Mat3xMat3(sita1, v0, sita1v0);
	Mat3xMat3(v1, sita0, v1sita0);
    for (int i = 0; i < 3; i++)v_fk_b[i] = v0[i] + sita0v0[i] * 0.5 + (sita1v0[i] + v1sita0[i]) / 12.0;
    double v_fk_n[3] = { 0.0 }, Cbn[9] = { 0.0 }, xi10[3] = { 0.0 }, xi10X[9] = { 0.0 };
    QuaToDCM(imu_pos->q_1, Cbn);
    for (int i = 0; i < 3; i++)xi10[i] = (omega_ie_n[i] + omega_en_n[i]) * dt;
    Antisymmetric_matrix(xi10, xi10X);
    double I[9] = { 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 }, Ixi[9] = { 0.0 }, Cv[3] = { 0.0 };
    for (int i = 0; i < 9; i++)Ixi[i] = I[i] - 0.5 * xi10X[i];
    MatrixMultiply(3, 3, 3, 1, Cbn, v_fk_b,Cv);
    MatrixMultiply(3, 3, 3, 1, Ixi, Cv, v_fk_n);
    //计算重力/哥氏积分项
    double v_g_n[3] = { 0.0 }, a_gc[3] = { 0.0 }, omegaieen[3] = { 0.0 }, omegav[3] = { 0.0 };
    for (int i = 0; i < 3; i++)omegaieen[i] = 2 * omega_ie_n[i] + omega_en_n[i];
    Mat3xMat3(omegaieen, v_half, omegav);
    for (int i = 0; i < 3; i++)a_gc[i] = g_p_n[i] - omegav[i], v_g_n[i] = a_gc[i] * dt;
    //更新速度
    for (int i = 0; i < 3; i++)imu_pos->v_0[i] = imu_pos->v_1[i] + v_fk_n[i] + v_g_n[i];
}


/**
* @brief          位置修正
* @param[in]      dt          const double              两次IMU数据时间间隔
* @param[in/out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return         void
* @note           nothing.
*/
void PositionUpdate(IMU_POS* imu_pos, const double dt)
{
    //计算中间时刻的速度
    double v_half[3] = { 0.0 };
    for (int i = 0; i < 3; i++)v_half[i] = 0.5 * (imu_pos->v_0[i] + imu_pos->v_1[i]);
    //更新高程，并计算中间时刻的高程
    imu_pos->geo_0.height = imu_pos->geo_1.height - v_half[2] * dt;
    double h_half = 0.5 * (imu_pos->geo_0.height + imu_pos->geo_1.height);
    //更新纬度，并计算中间时刻的纬度
    imu_pos->geo_0.latitude = imu_pos->geo_1.latitude + v_half[0] * dt / (RM(imu_pos->geo_1.latitude) + h_half);
    double lat_half = 0.5 * (imu_pos->geo_0.latitude + imu_pos->geo_1.latitude);
    //更新经度
    imu_pos->geo_0.longitude = imu_pos->geo_1.longitude + v_half[1] * dt / ((RN(imu_pos->geo_1.latitude) + h_half) * cos(lat_half));
}


/**
* @brief       姿态修正
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void AttitudeUpdate(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt)
{
    GEOCOOR pos_half_blh;
    //重新计算中间时刻的位置和速度
    double v_half[3] = { 0.0 };
    for (int i = 0; i < 3; i++)
    {
		pos_half_blh.blh[i] = 0.5 * (imu_pos->geo_1.blh[i] + imu_pos->geo_0.blh[i]);
        v_half[i] = 0.5 * (imu_pos->v_1[i] + imu_pos->v_0[i]);
    }
    double Rm = RM(pos_half_blh.latitude), Rn = RN(pos_half_blh.latitude);
    double omega_ie_n[3] = { 0.0 }, omega_en_n[3] = { 0.0 }, g_p_n[3] = { 0.0 };
    Omega_ie_n(pos_half_blh.latitude, omega_ie_n);
    G_p_n(&pos_half_blh, g_p_n);
    Omega_en_n(&pos_half_blh, v_half, omega_en_n);
    //计算 n 系的旋转四元数
    double xi[3] = { 0.0 }, q_n_01[4] = { 0.0 };
    for (int i = 0; i < 3; i++)xi[i] = -(omega_ie_n[i] + omega_en_n[i]) * dt;
    EquRotToQua(xi, q_n_01);
    //计算 b 系的旋转四元数(补偿圆锥效应)
    double sita0[3] = { imu_data0->x_gyo * dt,imu_data0->y_gyo * dt,imu_data0->z_gyo * dt };
    double sita1[3] = { imu_data1->x_gyo * dt,imu_data1->y_gyo * dt,imu_data1->z_gyo * dt };
    double Phi[3] = { 0.0 }, q_b_10[4] = { 0.0 }, sita1sita0[3] = { 0.0 };
    Mat3xMat3(sita1, sita0, sita1sita0);
    for (int i = 0; i < 3; i++)Phi[i] = sita0[i] + sita1sita0[i] / 12.0;
    EquRotToQua(Phi, q_b_10);
    //更新姿态
    QuaxQua(q_n_01, imu_pos->q_1, imu_pos->q_0);
    QuaxQua(imu_pos->q_0, q_b_10, imu_pos->q_0);
    QuaNorm(imu_pos->q_0);
}


/**
* @brief       惯导导航解算
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[in]   is_ZUPT     const bool                是否进行零速修正
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void IMUNav(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt,const bool is_ZUPT)
{
    VelocityUpdate(imu_data0, imu_data1, imu_pos, dt);
    PositionUpdate(imu_pos, dt);
    AttitudeUpdate(imu_data0, imu_data1, imu_pos, dt);
    //imu_pos->v_0[2] = 0.0;
    if (is_ZUPT)VelocityZUPT(imu_pos);
    for (int i = 0; i < 3; i++)
    {
		imu_pos->geo_2.blh[i] = imu_pos->geo_1.blh[i];
		imu_pos->geo_1.blh[i] = imu_pos->geo_0.blh[i];
    }
    memcpy(imu_pos->v_2, imu_pos->v_1, 3 * sizeof(double));
    memcpy(imu_pos->v_1, imu_pos->v_0, 3 * sizeof(double));
    memcpy(imu_pos->q_1, imu_pos->q_0, 4 * sizeof(double));
}