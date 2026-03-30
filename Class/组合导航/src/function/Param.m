% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
% -------------------------------------------------------------------------
function param = Param()
    param.D2R = pi / 180;
    param.R2D = 180 / pi;

    % WGS84 Ellipsoidal model parameters
    % NOTE:如果使用其他椭球模型需要修改椭球参数
    param.WGS84_WIE = 7.2921151467E-5;       % 地球自转角速度
    param.WGS84_F   = 0.0033528106647474805; % 扁率
    param.WGS84_RA  = 6378137.0000000000;    % 长半轴a
    param.WGS84_RB  = 6356752.3142451793;    % 短半轴b
    param.WGS84_GM0 = 398600441800000.00;    % 地球引力常数
    param.WGS84_E1  = 0.0066943799901413156; % 第一偏心率平方
    param.WGS84_E2  = 0.0067394967422764341; % 第二偏心率平方 

end