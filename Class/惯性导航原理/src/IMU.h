#pragma once

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
#include<vector>
#include<fstream>
#include<cmath>
#include<string>
#include"IMU.h"
#include"struct.h"


using namespace std;

/**
* @brief       均值计算
* @param[in]   data  const vector<T>  待求均值数据
* @return      double                 数据均值
* @note        使用前需要明确vector数据类型
*/
template <typename T>
double mean(const vector<T>data);


/**
* @brief       子午圈曲率半径计算
* @param[in]   lat  const double  纬度[rad]
* @return      double             子午圈半径[m]
* @note        nothing.
*/
double RM(const double lat);


/**
* @brief       卯酉圈曲率半径计算
* @param[in]   lat  const double  纬度[rad]
* @return      double             卯酉圈半径[m]
* @note        nothing.
*/
double RN(const double lat);


/**
* @brief       n系地球自转角速度计算
* @param[in]   lat       const double   纬度[rad]
* @param[out]  omega_ie  double[]       n系地球自转角速度[rad/s,rad/s,rad/s]
* @return      void
* @note        nothing.
*/
void Omega_ie_n(const double lat, double w[]);


/**
* @brief       n系位移角速度计算
* @param[in]   geo  const GEOCOOR*   大地坐标系坐标[rad,rad,m]
* @param[in]   v    const double[]   n系速度NED[m/s,m/s,m/s]
* @param[out]  w    double[]         n系位移角速度[rad/s,rad/s,rad/s]
* @return      void
* @note        注意v是NED速度
*/
void Omega_en_n(const GEOCOOR* geo,const double v[], double w[]);


/**
* @brief       n系重力加速度计算
* @param[in]   geo  const GEOCOOR*   大地坐标系坐标[rad,rad,m]
* @param[out]  g_p  double[]         n系重力加速度[m/s²,m/s²,m/s²]
* @return      void
* @note        有两种算法，一种是精密公式，另一种是二阶展开式
*/
void G_p_n(const GEOCOOR* geo, double g_p[]);


/**
* @brief       积分
* @param[in]   data    const vector<double>   数据
* @param[in]   time    const vector<double>   数据时间
* @return      double                         数据积分结果
* @note        保证data和time长度一致
*/
double Integral(const vector<double>data, const vector<double>time); 


/**
* @brief       读取IMU原始数据文件(整个文件保存)
* @param[in]   filename    const string  IMU数据文件路径
* @param[out]  imu_data    IMU_DATA*     IMU数据结构体
* @return      int         0表示读取成功；-1表示失败
* @note        nothing.
*/
int Read_IMU_File(const string filename,IMU_DATA* imu_data); 


/**
* @brief       IMU数据读取函数（@静态粗对准入口）
* @param[in]   filename  const string     IMU数据文件路径
* @return      int       0表示运行成功；-1表示失败
* @note        nothing.
*/
int Read_IMU_File(const string filename); 


/**
* @brief       加速度计标定（六位置法标定）
* @param[in]   data_set    const IMU_DATA*  IMU数据集（六位置法）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void ACC_Cal(const IMU_DATA* imu_data_set, IMU_ERROR* imu_error);


/**
* @brief       陀螺仪计标定
* @param[in]   fwd_data    const IMU_DATA*  IMU前向旋转数据
* @param[in]   rev_data    const IMU_DATA*  IMU后向旋转数据
* @param[in]   index       const int        轴索引（0：x轴；1：y轴；2：z轴）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void GYR_Cal(const IMU_DATA* fwd_data, const IMU_DATA* rev_data, const int index, IMU_ERROR* imu_error);  


/**
* @brief       利用加速度计六位置法静态数据标定陀螺零偏
* @param[in]   imu_data    const IMU_DATA*  IMU数据结构体数组（六位置法）
* @param[out]  imu_error   IMU_ERROR*       IMU误差结构体
* @return      void
* @note        nothing.
*/
void GYR_Bias(const IMU_DATA* imu_data_set, IMU_ERROR* imu_error);  


/**
* @brief       误差补偿
* @param[in]   imu_error   const IMU_ERROR*  IMU误差结构体
* @param[out]  imu_data    IMU_DATA*         待补偿的IMU数据结构体
* @return      void
* @note        nothing.
*/
void Compensation(IMU_DATA* imu_data, const IMU_ERROR* imu_error);  


/**
* @brief       静态粗对准
* @param[in]   static_imu_data    const STATIC_IMU_DATA*  静态IMU数据
* @param[out]  M                  double*                 初始姿态角（roll,pitch,yaw）[rad,rad,rad]
* @return      void
* @note        nothing.
*/
void StaticAlignment(const STATIC_IMU_DATA* static_imu_data, double* M);


/**
* @brief       Allan方差计算
* @param[in]   data        const vector<double>  待计算数据
* @param[in]   tau0        const double          采样时间间隔
* @param[in]   filename    const string          输出文件名
* @return      void
* @note        nothing.
*/
void AllanVariance(const vector<double>imu_data, const double tau0, const string filename);  


/**
* @brief       解析IMU数据行
* @param[in]   line      const string     IMU数据行
* @param[out]  imu_data  STATIC_IMU_DATA* IMU数据结构体
* @return      void
* @note        此次使用STATIC_IMU_DATA来存储单条IMU数据，避免重复定义结构体
*/
void IMU_Data(const string line, STATIC_IMU_DATA* static_imu_data);


/**
* @brief       四元数转欧拉角
* @param[in]   q   const double[]    四元数
* @return      EulerAngle            欧拉角结构体
* @note        nothing.
*/
EulerAngle QuaToEuler(const double q[]);


/**
* @brief       欧拉角转四元数
* @param[in]   ea  const EulerAngle  欧拉角结构体
* @param[out]  q   const double[]    四元数
* @return      void
* @note        nothing.
*/
void EulerToQua(const EulerAngle ea, double q[]);


/**
* @brief       欧拉角转方向余弦矩阵
* @param[in]   ea  const EulerAngle  欧拉角结构体
* @param[out]  C   double[]          方向余弦矩阵
* @return      void
* @note        nothing.
*/
void EulerToDCM(const EulerAngle ea, double C[]);


/**
* @brief           四元数归一化
* @param[in/out]   q      double[]     四元数
* @return          void
* @note            nothing.
*/
void QuaNorm(double q[]);


/**
* @brief       等效旋转矢量转四元数
* @param[in]   equRot  const double[]  等效旋转矢量
* @param[out]  q       double[]        四元数
* @return      void
* @note        nothing.
*/
void EquRotToQua(const double equRot[], double q[]);


/**
* @brief       四元数转方向余弦矩阵
* @param[in]   q   const double[]  四元数
* @param[out]  C   double[]        方向余弦矩阵
* @return      void
* @note        nothing.
*/
void QuaToDCM(const double q[], double C[]);


/**
* @brief       读取参考真值文件
* @param[in]   filename  const string     参考真值文件路径
* @return      void
* @note        nothing.
*/
void ReadTRUE(const string filename);


/**
* @brief       IMU初始化
* @param[in]   M          const double[]   初始姿态角（roll,pitch,yaw）[rad,rad,rad]
* @param[out]  imu_pos    IMU_POS*         IMU位置姿态
* @return      void
* @note        nothing.
*/
void IMUInit(const double M[], IMU_POS* imu_pos);


/**
* @brief       零速检测
* @param[in]   geo        const GEOCOOR*           大地坐标系坐标[rad,rad,m]
* @param[in]   imu_data   const STATIC_IMU_DATA*   单历元IMU数据
* @return      bool		  是否为零速，true表示是零速, false表示非零速
* @note        nothing.
*/
bool ZeroVelocityDetect(const GEOCOOR* geo, const STATIC_IMU_DATA* imu_data);


/**
* @brief       速度修正
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void VelocityZUPT(IMU_POS* imu_pos);


/**
* @brief       速度修正
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void VelocityUpdate(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt);


/**
* @brief          位置修正
* @param[in]      dt          const double              两次IMU数据时间间隔
* @param[in/out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return         void
* @note           nothing.
*/
void PositionUpdate(IMU_POS* imu_pos,const double dt);


/**
* @brief       姿态修正
* @param[in]   imu_data0   const STATIC_IMU_DATA*    当前历元IMU数据
* @param[in]   imu_data1   const STATIC_IMU_DATA*    上个历元IMU数据
* @param[in]   dt          const double              两次IMU数据时间间隔
* @param[out]  imu_pos     IMU_POS*                  解算得到的IMU状态结构体
* @return      void
* @note        nothing.
*/
void AttitudeUpdate(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt);


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
void IMUNav(const STATIC_IMU_DATA* imu_data0, const STATIC_IMU_DATA* imu_data1, IMU_POS* imu_pos, const double dt,const bool is_ZUPT);