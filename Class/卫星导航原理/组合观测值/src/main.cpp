#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;

#define GPS_L1 1575.42e6    // L1Ƶ��
#define GPS_L2 1227.60e6    // L2Ƶ��
#define BDS_B1 1561.098e6    // B1Ƶ��
#define BDS_B3 1268.52e6    // B3Ƶ��
#define c_LIGHT 299792458.0    // ����

// �����۲����Ͳ���ȡ�ؼ�����
void ObsIndex(const vector<string>& types/*�۲�ֵ����*/, char sys, int& idxP1, int& idxP2, int& idxL1, int& idxL2) 
{
    idxP1 = idxP2 = idxL1 = idxL2 = -1;    //��ʼ������
    for (int j = 0; j < (int)types.size(); ++j) {
        const string& t = types[j];
        if (sys == 'G') {
            if (t == "C1C") idxP1 = j;
            if (t == "C2W") idxP2 = j;
            if (t == "L1C") idxL1 = j;
            if (t == "L2W") idxL2 = j;
        }
        else if (sys == 'C') {
            if (t == "C2I") idxP1 = j;
            if (t == "C6I") idxP2 = j;
            if (t == "L2I") idxL1 = j;
            if (t == "L6I") idxL2 = j;
        }
    }
}
double extractObs(const string& line, int index)     //��ȡ�۲�ֵ
{
	int start = index * 16 + 3;    //�۲�ֵ��ʼλ��
    if (start + 14 > (int)line.size()) return 0.0;
    string obs = line.substr(start, 14);
    return atof(obs.c_str());
}

int main() {
    ifstream ifs("JFNG00CHN_R_20200010000_01D_30S_MO.20o"); 
	ofstream out("result.txt"); // �������ļ�
    string line;
    vector<string> obsTypesG, obsTypesC;
    while (getline(ifs, line))
    {
        if (line.find("SYS / # / OBS TYPES") != string::npos)
        {
            char sys = line[0];
			if (sys != 'G' && sys != 'C') continue;
			int n = stoi(line.substr(3, 3));
			vector<string> tempTypes;    //��ʱ�洢�۲�ֵ����
            int count = 0;
            do {
                for (int i = 0; i < 13 && count < n; ++i, ++count)     //һ������13��
                {
					string t = line.substr(7 + i * 4, 3);    //��ȡ�۲�ֵ����
                    tempTypes.push_back(t);
                }
                if (count < n) getline(ifs, line);
            } while (count < n);
            if (sys == 'G') obsTypesG = tempTypes;
            if (sys == 'C') obsTypesC = tempTypes;
        }
        else if (line.find("END OF HEADER") != string::npos) {
            break;
        }
    }
    int idxP1G, idxP2G, idxL1G, idxL2G, idxP1C, idxP2C, idxL1C, idxL2C;
    ObsIndex(obsTypesG, 'G', idxP1G, idxP2G, idxL1G, idxL2G);
    ObsIndex(obsTypesC, 'C', idxP1C, idxP2C, idxL1C, idxL2C);
    while (getline(ifs, line)) {
		if (line.empty() || line[0] != '>') continue;   //�����ǹ۲�������
		int numSats = stoi(line.substr(32, 3));    //��������
		int hour = stoi(line.substr(13, 2));    //��ȡʱ
		int min = stoi(line.substr(16, 2));    //��ȡ��
		double sec = stof(line.substr(19, 9));    //��ȡ��
        for (int i = 0; i < numSats; ++i)
        {
			getline(ifs, line);    //��ȡһ�����ǹ۲�����
            if (line.size() < 3) continue;   
            string prn = line.substr(0, 3);
            char sys = prn[0];
            if (sys != 'G' && sys != 'C') continue;
            int idxP1 = -1, idxP2 = -1, idxL1 = -1, idxL2 = -1;
            double f1 = 0, f2 = 0;
			vector<string>* obsTypes = nullptr;    //�۲�ֵ����ָ��
			//����ϵͳѡ��Ƶ�ʺ�����
            if (sys == 'G')
            {
                idxP1 = idxP1G; idxP2 = idxP2G;
                idxL1 = idxL1G; idxL2 = idxL2G;
                f1 = GPS_L1; f2 = GPS_L2;
                obsTypes = &obsTypesG;
            }
            else if (sys == 'C')
            {
                idxP1 = idxP1C; idxP2 = idxP2C;
                idxL1 = idxL1C; idxL2 = idxL2C;
                f1 = BDS_B1; f2 = BDS_B3;
                obsTypes = &obsTypesC;
            }
            if (idxP1 < 0 || idxP2 < 0 || idxL1 < 0 || idxL2 < 0) continue;
			//����������ȡ�۲�ֵ
            double L1 = extractObs(line, idxL1);
            double L2 = extractObs(line, idxL2);
            double P1 = extractObs(line, idxP1);
            double P2 = extractObs(line, idxP2);
            if (L1 == 0 || L2 == 0 || P1 == 0 || P2 == 0) continue;
			//����MW��GF
            double lambda1 = c_LIGHT / f1;
            double lambda2 = c_LIGHT / f2;
            double L1_m = L1 * lambda1;
            double L2_m = L2 * lambda2;
            double mw = (f1*L1_m - f2*L2_m)/(f1-f2) - (f1 * P1 + f2 * P2)/ (f1 + f2);
            double gf = L1_m - L2_m;
			//���浽�ļ�
            out << hour << " " << min << " " << setprecision(1) << fixed << sec << " ";
			out << prn << " " << setprecision(3) << fixed << mw << " " << gf << endl;
            cout << prn << fixed << setprecision(3) << " MW: " << mw << " GF: " << gf << endl;
        }
    }
	ifs.close();
    out.close();
    return 0;
}