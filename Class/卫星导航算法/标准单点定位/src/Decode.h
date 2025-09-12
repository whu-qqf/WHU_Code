#pragma once
#include"struct.h"
#include"const.h"
#define POLYCRC32 0xEDB88320u

unsigned int UI4(unsigned char* p);    //��ȡ4�ֽ��޷�������
int I4(unsigned char* p);    //��ȡ4�ֽ��з�������
short I2(unsigned char* p);    //��ȡ2�ֽ��з�������
float F4(unsigned char* p);    //��ȡ4�ֽڸ�����
double D8(unsigned char* p);    //��ȡ8�ֽ�˫���ȸ�����
unsigned short UI2(unsigned char* p);    //��ȡ2�ֽ��޷�������
unsigned int Crc32(unsigned char* buff, int len);    //����CRC32У��
void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs);    //����۲�����
void DecodeGpsEph(unsigned char* data, int len, GPSEPHREC geph[]);    //����GPS��������
void DecodeBdsEph(unsigned char* data, int len, GPSEPHREC beph[]);    //���뱱����������
void DecodePsrPos(unsigned char* data, int len, POSRES* pos);    //���붨λ�������
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);    //����NovAtel OEM7����