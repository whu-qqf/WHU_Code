#pragma once

void PrintMatrix( int rows, int cols, const double* matrix, int width = 10, int precision = 4);
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
void MatrixAdd(int m1, int n1, const double M1[], const double M2[], double M3[]);
void MatrixTranspose(int rows, int cols, const double* matrix, double* transposed);
void MatrixInverse(int n, const double* matrix, double* inverse);
void MatrixInverse(int m, int n, double* matrix, double* inverse);
void MatrixExtr(int rows, int cols, int a[], int a_len,  /* 被删除的行数组及其长度*/int b[], int b_len, double m[], double n[]);
void MatrixSub(int m1, int n1, const double M1[], const double M2[], double M3[]);
