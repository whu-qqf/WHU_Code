#pragma once

/**
* @file     const.h
* @brief    这个文件给出常量定义
* @details  这个文件给出项目中常用的常量定义
* @author   Qifeng Qin. Email: 2023302143029@whu.edu.cn
* @date     2025/11/29
* @version  0.0.01
* @par      Copyright(c) 2012-2021 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
* @par      History:
*           2025/11/29,Qifeng Qin, Modify and finish.          
*/

#include<cmath>
#define R_WGS84  6378137.0                /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563        /* Flattening; WGS-84   */
#define e2_WGS84  0.00669437999013        /* Square eccentricity; WGS-84 */
#define C_Light 299792458.0               /*光速 [m/s]*/
#define PI 3.14159265358979323846         /*圆周率*/
#define GM_GPS 3.986005E14                /*GPS引力常数*/
#define OMEGA_E 7.292115E-5               /*地球自转角速度 [rad/s]*/
#define acc_scale 1.5258789063E-06        /*星网宇达加速度计系数*/
#define gyo_scale 1.0850694444E-07        /*星网宇达陀螺仪系数*/
#define novatel_acc_scale 2.0E-8          /*NoVatel加速度计系数*/
#define novatel_gyo_scale 1.0E-9          /*NoVatel陀螺仪系数*/
#define chcn_accel_scale 100/655360       /*华测加速度计系数*/
#define chcn_gyo_scale 100/160849.543863  /*华测陀螺仪系数*/
#define LOCAL_LAT 30.531651244            /*实验实纬度 [deg]*/
#define CAR_LAT 30.5279708506             /*实验车辆纬度 [deg]*/
#define g 9.7936174                       /*实验实重力加速度 [m/s²]*/
#define gama_a 9.7803267715               /*重力加速度参数γa*/
#define gama_b 9.8321863685               /*重力加速度参数γb*/
#define B_WGS84 6356752.3141              /*地球短半轴 [m]*/

