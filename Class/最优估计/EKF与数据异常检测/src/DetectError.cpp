#include<cmath>
#include"Matrix.h"
#include"const.h"
#include"DetectError.h"

double EpochError(double V[], double D[], int l)
{
	double VT[1 * MAXCHANNUM] = { 0 }, VTD[1 * MAXCHANNUM] = { 0 }, VTDV[1] = { 0 };
	MatrixTranspose(MAXCHANNUM, 1, V, VT);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, VT, D, VTD);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, VTD, V, VTDV);
	return VTDV[0] / l * 1.0;
}
double SingleError(double V[], double D[], int i)
{
	double C[MAXCHANNUM] = { 0 }, CT[MAXCHANNUM] = { 0 }, CTD[1 * MAXCHANNUM] = { 0 }, CTDV[1] = { 0 }, CTDC[1] = { 0 };
	C[i] = 1.0;
	MatrixTranspose(MAXCHANNUM, 1, C, CT);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, CT, D, CTD);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, CTD, V, CTDV);
	MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, CTD, C, CTDC);
	double v = CTDV[0];
	double sigma2 = CTDC[0];
	return abs(v) / sqrt(sigma2) * 1.0;
}