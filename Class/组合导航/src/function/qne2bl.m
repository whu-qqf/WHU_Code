% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Qijin Chen
% Contact : chenqijin@whu.edu.cn
%    Date : Nov. 2019;
% -------------------------------------------------------------------------
% QNE2BL Compute latitude and longitude from the quaternion q_ne
%	[lat, lon] = qne2bl(q_ne)
%
% INPUTS:
%	q_ne = the quaternion q_ne
% OUTPUTS:
%	1. lat = latitude in radians
%	2. lat = longitude in radians
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005. pp.23
% -------------------------------------------------------------------------
function [lat, lon] = qne2bl(qne)
    lat = -2 * atan(qne(3) / qne(1)) - pi / 2;
    lon = 2 * atan2(qne(4), qne(1));
end