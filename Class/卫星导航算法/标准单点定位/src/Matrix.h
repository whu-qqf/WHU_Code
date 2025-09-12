#pragma once

void PrintMatrix(int rows, int cols, const double* matrix, int width = 10, int precision = 4);    // ��ӡ����
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);    // ����˷�
void MatrixAdd(int m1, int n1, const double M1[], const double M2[], double M3[]);    // ����ӷ�
void MatrixTranspose(int rows, int cols, const double* matrix, double* transposed);    // ����ת��
bool MatrixInverse(int n, const double* matrix, double* inverse);    // ��������
bool MatrixInverse(int m, int n, double* matrix, double* inverse);    // ����ֿ�����
void MatrixExtr(int rows, int cols, int a[], int a_len,  /* ��ɾ���������鼰�䳤��*/int b[], int b_len, double m[], double n[]);    // ��������ʽ
void MatrixSub(int m1, int n1, const double M1[], const double M2[], double M3[]);    // �������
