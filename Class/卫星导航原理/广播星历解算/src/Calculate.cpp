#include"Calculate.h"
#include<cmath>
#include<vector>
#include<ctime>
#include<chrono>

#define pi 4*atan(1)

using namespace std;

vector<Satellite> Calculate(vector<Ephemeris> alldata)
{

	vector<Satellite> positions; // 存储卫星位置的向量
	for (int i = 0; i < alldata.size(); i++)
	{
        Satellite* Sat = new Satellite();
		Ephemeris eph = alldata[i];
		Sat->satellitename = eph.satellitename;
		double t = eph.time;
		Sat->time = t;
        double n_0;
        if (eph.satellitename.find("G") != string::npos) {
            n_0 = sqrt(GM) * 1.0 / (eph.sqrt_A * eph.sqrt_A * eph.sqrt_A); // 计算平均角速度
        }
        if (eph.satellitename.find("C") != string::npos) {
            n_0 = sqrt(BDS_GM) * 1.0 / (eph.sqrt_A * eph.sqrt_A * eph.sqrt_A); // 计算平均角速度
        }
		double n = n_0 + eph.delta_n; // 计算修正后的平均角速度

        if (eph.satellitename.find("C") != string::npos) t -= 14;
        double t_k = t - eph.toe;
        if (t_k > 302400) t_k -= 604800; // 确保 t_k 在 [-302400, 302400] 范围内
        if (t_k < -302400) t_k += 604800;

		double A = eph.sqrt_A * eph.sqrt_A; // 计算轨道长半轴
		double M = eph.M0 + n * t_k*1.0; // 计算平近点角
		double E_0 = M; //计算偏近点角
        double E=M;
        do {
            E_0 = E;
			E = M+eph.e*sin(E_0); // 迭代计算偏近点角
			if (fabs(E - E_0) < 1e-8) break; // 收敛条件
        } while (true);
        double sin_v = sqrt(1 - eph.e * eph.e) * sin(E) / (1 - eph.e * cos(E));
        double cos_v = (cos(E) - eph.e) / (1 - eph.e * cos(E));
        double f_t = atan2(sin_v, cos_v);
		double u_t = f_t + eph.omega;  
        double u =  u_t+ eph.Cuc * cos(2 * u_t) + eph.Cus * sin(2 * u_t);
        double r = A * (1 - eph.e * cos(E)) + eph.Crc * cos(2 * u_t) + eph.Crs * sin(2 * u_t);
        double i_ = eph.i0 + eph.i_dot * t_k + eph.Cic * cos(2 * u_t) + eph.Cis * sin(2 * u_t);
        double x_prime = r * cos(u);
        double y_prime = r * sin(u);
        double L = 0;
		if (eph.satellitename.find("G") != string::npos)
		{
            L = eph.omega0 + (eph.omega_dot - OMEGA_E) * t_k - OMEGA_E * eph.toe;
		}
		if (eph.satellitename.find("C") != string::npos)
		{
			L = eph.omega0 + (eph.omega_dot - BDS_OMEGA_E) * t_k - BDS_OMEGA_E * eph.toe;
		}
        if (eph.satellitename.find("C01") != string::npos || eph.satellitename.find("C02") != string::npos || eph.satellitename.find("C03") != string::npos || eph.satellitename.find("C04") != string::npos || eph.satellitename.find("C05") != string::npos)
        {
            L = eph.omega0 + (eph.omega_dot) * t_k - BDS_OMEGA_E * eph.toe; 
        }
        double x = x_prime * cos(L) - y_prime * cos(i_) * sin(L);
        double y = x_prime * sin(L) + y_prime * cos(i_) * cos(L);
        double z = y_prime * sin(i_);
        double p = BDS_OMEGA_E * t_k;
        double f = -5 * pi / 180.0;

        double A_x = x, B = y, C = z;

        if (eph.satellitename.find("C01") != string::npos || eph.satellitename.find("C02") != string::npos || eph.satellitename.find("C03") != string::npos || eph.satellitename.find("C04") != string::npos || eph.satellitename.find("C05") != string::npos)
        {
            x = A_x * cos(p) + B * sin(p) * cos(f) + C * sin(f) * sin(p);
			y = -A_x * sin(p) + B * cos(p) * cos(f) + C * sin(f) * cos(p);
			z = -B * sin(f) + C * cos(f);
        }
        Sat->x = x;
        Sat->y = y;
        Sat->z = z;

        positions.push_back(*Sat);
        delete Sat;
	}
	return positions;
}

