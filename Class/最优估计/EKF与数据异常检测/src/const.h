#pragma once

#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */

#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */

#define GPST_BDT_SEC  14         /* GPS时与北斗时的差值[s] */
#define GPST_BDT_WEEK  1356         /* GPS时与北斗时的差值[week] */
#define MAXCHANNUM 36
#define MAXGPSNUM  32
#define MAXBDSNUM 63
#define MAXRAWLEN 40960
#define FILEMODE 1
#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define C_Light 299792458.0      
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */

#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)
#define  FG1_BDS  1561.098E6               /* B1信号的基准频率 */
#define  FG3_BDS  1268.520E6               /* B3信号的基准频率 */
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)       // 波长

#define PI 3.1415926535898
#define GM_BDS 3.986004418E14
#define GM_GPS 3.986005E14
#define OMEGAE_GPS 7.2921151467E-5
#define OMEGAE_BDS 7.2921150E-5

//#define F -4.442807633E-10

#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define E_WGS84  (F_WGS84 * (2 - F_WGS84)) /* Square of eccentricity */

#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define E_CGS2K  (F_CGS2K * (2 - F_CGS2K)) /* Square of eccentricity */
