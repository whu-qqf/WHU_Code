% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% getGravity Get normal gravity at given position
%
% INPUTS: blh = [latitude; longitude; height] in [radians, radians, meters]
% OUTPUTS: gravity in [m/s^2]
% 
% Reference:  equations (4.78a) and (4.79) on page 110 of "Geodesy" by Torge W. and Müller J. (de Gruyter, 2012).
% -------------------------------------------------------------------------
function g = getGravity(blh)
    sinphi = sin(blh(1));
    sin2 = sinphi * sinphi;
    sin4 = sin2 * sin2;

    % normal gravity at equator, 赤道处正常重力
    gamma_a = 9.7803267715;
    % series expansion of normal gravity at given latitude, 给定纬度处正常重力的级数展开
    gamma_0 = gamma_a * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin4 + 0.0000001262 * sin2 * sin4 + 0.0000000007 * sin4 * sin4);
    % changes of normal gravity with height, 正常重力随高度变化
    g = gamma_0 - (3.0877e-6 - 4.3e-9 * sin2) * blh(3) + 0.72e-12 * blh(3) * blh(3);
end