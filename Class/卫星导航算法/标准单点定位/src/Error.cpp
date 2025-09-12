#include"Error.h"
#include"const.h"
#include<cmath>

/**
* @brief Hopefield对流层模型
* @param H 高度（单位：米）
* @param Elev 卫星仰角（单位：度）
* @return 返回对流层延迟（单位：米）
*/
double Hopfield(const double H, const double Elev)
{
	double RH, RH0, H0, p, p0, T, T0, e, hw, hd, Kw, Kd, delta_Trop;
	if (H < 0 || H>3e4)return 0;
	H0 = 0, T0 = 15 + 273.16, p0 = 1013.25, RH0 = 0.5, hw = 11000;
	RH = RH0 * exp(-0.0006396 * (H - H0));
	p = p0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	T = T0 - 0.0065 * (H - H0);
	e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	hd = 40136 + 148.72 * (T0 - 273.16);
	Kw = 155.2E-7 * 4810 * e * (hw - H) / (T * T);
	Kd = 155.2E-7 * p * (hd - H) / T0;
	delta_Trop = Kd / sin(sqrt(Elev * Elev + 6.25) * PI / 180.0) + Kw / sin(sqrt(Elev * Elev + 6.25) * PI / 180.0);
	return delta_Trop;
}

/**
* @brief 粗差与周跳探测
* @param Obs 指向EPOCHOBSDATA结构体的指针，包含卫星观测数据
*/ 
void DetectOutlier(EPOCHOBSDATA* Obs)
{
	MWGF CurComObs[MAXCHANNUM];
	double f1=0.0, f2=0.0;
	for (int i = 0; i < Obs->SatNum; i++)
	{

		CurComObs[i].Prn = Obs->SatObs[i].Prn;
		CurComObs[i].Sys = Obs->SatObs[i].System;
		if (fabs(Obs->SatObs[i].P[0]) < 1e-5 || fabs(Obs->SatObs[i].P[1]) < 1e-5 || fabs(Obs->SatObs[i].L[0]) < 1e-5 || fabs(Obs->SatObs[i].L[1]) <1e-5 )    //观测值不存在，无效
		{
			Obs->SatObs[i].Valid = false;
			continue;
		}
		if (CurComObs[i].Sys == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; }
		else if (CurComObs[i].Sys == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
		//计算组合观测值
		CurComObs[i].MW = (Obs->SatObs[i].L[0] * f1 - Obs->SatObs[i].L[1] * f2) / (f1 - f2) - (Obs->SatObs[i].P[0] * f1 + Obs->SatObs[i].P[1] * f2) / (f1 + f2);
		CurComObs[i].GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
		CurComObs[i].PIF = (f1 * f1 * Obs->SatObs[i].P[0] - f2 * f2 * Obs->SatObs[i].P[1]) / (f1 * f1 - f2 * f2);
		CurComObs[i].n = 1;
		Obs->SatObs[i].Valid = true;
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (CurComObs[i].Prn == Obs->ComObs[j].Prn && CurComObs[i].Sys == Obs->ComObs[j].Sys)    //如果当前卫星的组合观测值已经存在于组合观测数组中
			{
				double dmw = fabs(CurComObs[i].MW - Obs->ComObs[j].MW);
				double dgf = fabs(CurComObs[i].GF - Obs->ComObs[j].GF);
				if (dmw < 3.0 && dgf < 0.05)    //如果组合观测值与已有的组合观测值差异小于阈值，则说明无粗差，更新组合观测值、平滑计数
				{
					CurComObs[i].n += Obs->ComObs[j].n;
					CurComObs[i].MW = (CurComObs[i].MW + Obs->ComObs[j].MW*(CurComObs[i].n-1)) / CurComObs[i].n ;
					//CurComObs[i].PIF = (f1 * f1 * Obs->SatObs[i].P[0] - f2 * f2 * Obs->SatObs[i].P[1]) / (f1 * f1 - f2 * f2);
					Obs->SatObs[i].Valid = true;
				}
				else Obs->SatObs[i].Valid = false;    //如果组合观测值与已有的组合观测值差异大于阈值，则说明有粗差，标记当前卫星观测值为无效
				break;
			}
		}
	}
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		Obs->ComObs[i] = CurComObs[i];
	}
}

