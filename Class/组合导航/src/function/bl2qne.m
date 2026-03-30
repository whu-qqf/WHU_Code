% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Qijin Chen
% Contact : chenqijin@whu.edu.cn
%    Date : Nov. 2019;
% -------------------------------------------------------------------------
% QNE The quaternion corresponding to C_ne (transformation matrix from n-frame to e-frame) 
%	q_ne = qne(lat, lon)
%
% INPUTS:
%	1. lat = latitude in radians
%	2. lat = longitude in radians
% OUTPUTS:
%	q_ne = the quaternion q_ne
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005. pp.23
% -------------------------------------------------------------------------
function qne = bl2qne(lat, lon)
      s1 = sin(lon / 2);
      c1 = cos(lon / 2);
      s2 = sin(-pi / 4 - lat / 2);
      c2 = cos(-pi / 4 - lat / 2);

      qne = [c1 * c2;
            -s1 * s2;
            c1 * s2;
            c2 * s1];
end