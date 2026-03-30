#pragma once
#include"struct.h"
#include"const.h"
#define POLYCRC32 0xEDB88320u

unsigned int UI4(unsigned char* p);    //读取4字节无符号整数
int I4(unsigned char* p);    //读取4字节有符号整数
short I2(unsigned char* p);    //读取2字节有符号整数
float F4(unsigned char* p);    //读取4字节浮点数
double D8(unsigned char* p);    //读取8字节双精度浮点数
unsigned short UI2(unsigned char* p);    //读取2字节无符号整数
unsigned int Crc32(unsigned char* buff, int len);    //计算CRC32校验
void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs);    //解码观测数据
void DecodeGpsEph(unsigned char* data, int len, GPSEPHREC geph[]);    //解码GPS星历数据
void DecodeBdsEph(unsigned char* data, int len, GPSEPHREC beph[]);    //解码北斗星历数据
void DecodePsrPos(unsigned char* data, int len, POSRES* pos);    //解码定位结果数据
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);    //解码NovAtel OEM7数据