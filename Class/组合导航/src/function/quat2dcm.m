% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% quat2dcm Convert from quaternion to Euler angle
%
% INPUTS: quat = [w; x; y; z] (4 * 1)
% OUTPUTS: euler = [roll; pitch; heading] in radians
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005.
% -------------------------------------------------------------------------
function dcm = quat2dcm(quat)
    % get quaternion elements
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    dcm = zeros(3, 3);

    dcm(1, 1) = w * w + x * x - y * y - z * z;
    dcm(1, 2) = 2 * (x * y - w * z);
    dcm(1, 3) = 2 * (x * z + w * y);
    dcm(2, 1) = 2 * (x * y + w * z);
    dcm(2, 2) = w * w - x * x + y * y - z * z;
    dcm(2, 3) = 2 * (y * z - w * x);
    dcm(3, 1) = 2 * (x * z - w * y);
    dcm(3, 2) = 2 * (y * z + w * x);
    dcm(3, 3) = w * w - x * x - y * y + z * z;
end