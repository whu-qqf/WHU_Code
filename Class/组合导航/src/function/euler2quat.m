% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% euler2quat Convert from Euler angle to quaternion - NED system
%    quat = euler2quat()
%
% INPUTS: euler = [roll; pitch; heading] in radians
% OUTPUTS: quat = [w; x; y; z] (4 * 1)
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005.
% -------------------------------------------------------------------------
function quat = euler2quat(euler)
     quat = [cos(euler(1) / 2) * cos(euler(2) / 2) * cos(euler(3) / 2) + sin(euler(1) / 2) * sin(euler(2) / 2) * sin(euler(3) / 2);
             sin(euler(1) / 2) * cos(euler(2) / 2) * cos(euler(3) / 2) - cos(euler(1) / 2) * sin(euler(2) / 2) * sin(euler(3) / 2);
             cos(euler(1) / 2) * sin(euler(2) / 2) * cos(euler(3) / 2) + sin(euler(1) / 2) * cos(euler(2) / 2) * sin(euler(3) / 2);
             cos(euler(1) / 2) * cos(euler(2) / 2) * sin(euler(3) / 2) - sin(euler(1) / 2) * sin(euler(2) / 2) * cos(euler(3) / 2)];
end