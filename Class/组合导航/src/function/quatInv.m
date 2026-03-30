% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% quatInv inverse of quaternion
%
% INPUTS: quat = [w; x; y; z] (4 * 1)
% OUTPUTS: quat_inv = [w; x; y; z] (4 * 1)
% -------------------------------------------------------------------------
function quat_inv = quatInv(quat)
    % 输入： 四元数 quat, [w; x; y; z]
    % 输入： 四元数 quat_inv, [w; x; y; z]

    % get quaternion elements
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    % 计算四元数的模的平方
    norm_sq = w * w + x * x + y * y + z * z;
    if norm_sq == 0
        error('norm of quaternion is zero!!');
    end
    
    % 计算四元数的共轭
    q_conj = [w, -x, -y, -z];
    
    % 四元数的逆为共轭除以模的平方
    quat_inv = quatNormalized(q_conj / norm_sq);
end
