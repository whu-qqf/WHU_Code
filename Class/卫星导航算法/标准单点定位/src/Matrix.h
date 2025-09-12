#pragma once

void PrintMatrix(int rows, int cols, const double* matrix, int width = 10, int precision = 4);    // 打印矩阵
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);    // 矩阵乘法
void MatrixAdd(int m1, int n1, const double M1[], const double M2[], double M3[]);    // 矩阵加法
void MatrixTranspose(int rows, int cols, const double* matrix, double* transposed);    // 矩阵转置
bool MatrixInverse(int n, const double* matrix, double* inverse);    // 矩阵求逆
bool MatrixInverse(int m, int n, double* matrix, double* inverse);    // 矩阵分块求逆
void MatrixExtr(int rows, int cols, int a[], int a_len,  /* 被删除的行数组及其长度*/int b[], int b_len, double m[], double n[]);    // 矩阵行列式
void MatrixSub(int m1, int n1, const double M1[], const double M2[], double M3[]);    // 矩阵减法
