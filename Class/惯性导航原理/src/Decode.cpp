/**
* @file     Decode.cpp
* @brief    这个文件给出解码函数定义
* @details  UI4 读取4字节无符号整数函数\n
*           I4  读取4字节有符号整数函数\n
* 		    I2  读取2字节有符号整数函数\n
*           F4  读取4字节浮点数函数\n
*           D8  读取8字节双精度浮点数函数\n
* 		    UI2 读取2字节无符号整数函数\n
*           DecodeResultDat 解码结果文件数据函数\n
*           DecodeIMUDat    解码IMU原始数据文件数据函数
* @author   Qifeng Qin. Email: 2023302143029@whu.edu.cn
* @date     2025/11/29
* @version  0.0.01
* @par      Copyright(c) 2012-2021 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
* @par      History:
*           2025/11/29,Qifeng Qin, Modify and finish.
*/

#include"Decode.h"

/**
* @brief       读取4字节无符号整数
* @param[in]   p  unsigned char*  读取起始处指针
* @return      unsigned int       无符号整数
* @note        nothing.
*/
unsigned int UI4(unsigned char* p)
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}


/**
* @brief       读取4字节有符号整数
* @param[in]   p   unsigned char*  读取起始处指针
* @return      int                 有符号整数
* @note        nothing.
*/
int I4(unsigned char* p) 
{
	int u;
	memcpy(&u, p, 4);
	return u;
}


/**
* @brief       读取8字节双精度浮点数
* @param[in]   p  unsigned char*  读取起始处指针
* @return      double             双精度浮点数
* @note        nothing.
*/
double D8(unsigned char* p)
{
	double u;
	memcpy(&u, p, 8);
	return u;
}


/**
* @brief       读取2字节有符号整数
* @param[in]   p  unsigned char*  读取起始处指针.
* @return      shoort             有符号整数.
* @note        nothing.
*/
short I2(unsigned char* p)
{
	short u;
	memcpy(&u, p, 2);
	return u;
}


/**
* @brief       读取4字节单精度浮点数
* @param[in]   p  unsigned char*  读取起始处指针
* @return      float              单精度浮点数
* @note        nothing.
*/
float F4(unsigned char* p)
{
	float u;
	memcpy(&u, p, 4);
	return u;
}


/**
* @brief       读取2字节无符号整数
* @param[in]   p  unsigned char*  读取起始处指针
* @return      unsigned short     无符号整数
* @note        nothing.
*/
unsigned short UI2(unsigned char* p)
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}


/**
* @brief       解码IMU原始数据文件数据（示例数据所用函数）
* @param[in]   buff       unsigned char*     待解码数据缓冲区
* @param[out]  imu_data   STATIC_IMU_DATA*   单历元IMU数据结构体指针
* @return      void
* @note        nothing.
*/
void DecodeIMUDat(unsigned char* buff, STATIC_IMU_DATA* imu_data)
{
	imu_data->gpsTime.SecOfWeek = imu_data->secofweek = D8(buff);
	imu_data->x_gyo = D8(buff + 8) * 200;
	imu_data->y_gyo = D8(buff + 16) * 200;
	imu_data->z_gyo = D8(buff + 24) * 200;
	imu_data->x_acc = D8(buff + 32) * 200;
	imu_data->y_acc = D8(buff + 40) * 200;
	imu_data->z_acc = D8(buff + 48) * 200;
}


/**
* @brief       解码结果文件数据（示例数据所用函数）
* @param[in]   buff       unsigned char*     待解码数据缓冲区
* @param[out]  result     IMUResult*         单历元结果数据结构体指针
* @return      void
* @note        nothing.
*/
void DecodeResultDat(unsigned char* buff, IMUResult* result)
{
	result->gpsTime.SecOfWeek = D8(buff);
	result->geo.latitude = D8(buff + 8);
	result->geo.longitude = D8(buff + 16);
	result->geo.height = D8(buff + 24);
	result->velocity[0] = D8(buff + 32);
	result->velocity[1] = D8(buff + 40);
	result->velocity[2] = D8(buff + 48);
	result->attitude.roll = D8(buff + 56);
	result->attitude.pitch = D8(buff + 64);
	result->attitude.yaw = D8(buff + 72);
}