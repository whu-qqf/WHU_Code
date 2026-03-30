% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
% quatNormalized normalize quaternion
%
% INPUTS: quat = [w; x; y; z] (4 * 1)
% OUTPUTS: quat_normalized = [w; x; y; z] (4 * 1)
% -------------------------------------------------------------------------
function quat_normalized = quatNormalized(quat)
    quat_normalized = zeros(4, 1);
    norm = sqrt(quat(1) * quat(1) + quat(2) * quat(2) + quat(3) * quat(3) + quat(4) * quat(4));
    if norm ~= 0
        quat_normalized = quat / norm;
    end
end