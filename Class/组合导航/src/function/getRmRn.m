% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% getRmRn Get meridian Prime Vertical Radius
%
% INPUTS: lat: latitude in radians;
%         param: predefined Parma()
% OUTPUTS: [rm, rn] radius in meters
% -------------------------------------------------------------------------
function [rm, rn] = getRmRn(lat, param)
    tmp = sin(lat) * sin(lat);
    tmp = 1 - param.WGS84_E1 * tmp;
    sqrttmp = sqrt(tmp);

    rm = param.WGS84_RA * (1 - param.WGS84_E1) / (sqrttmp * tmp);
    rn = param.WGS84_RA / sqrttmp;
end