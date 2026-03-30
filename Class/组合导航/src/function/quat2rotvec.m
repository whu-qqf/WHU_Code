% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% quat2rotvec Convert from quaternion to rotation vector
%
% INPUTS: quat = [w; x; y; z] (4 * 1)
% OUTPUTS: rotvec = [x; y; z] (3 * 1)
% 
% Reference: Niu Xiaoji, Chen Qijin, 惯导讲义
% -------------------------------------------------------------------------
function rotvec = quat2rotvec(quat)
    % get quaternion elements
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    qv_norm = sqrt(x * x + y * y + z * z);

    if w == 0
        rotvec = pi * [x; y; z];
    elseif qv_norm < 1e-10
        rotvec = [0; 0; 0];
    else
        temp = 2 * atan(qv_norm / w);
        f = sin(temp / 2) / temp;
        rotvec = [x; y; z] / f;
    end
end
