% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2025.3.6
% -------------------------------------------------------------------------
% dcm2quat Convert from rotation matrix to quaternion
%
% INPUTS: dcm (3 * 3)
% OUTPUTS: quat = [w; x; y; z] (4 * 1)
%
% Reference: Niu Xiaoji, Chen Qijin, 惯导讲义
% -------------------------------------------------------------------------

function quat = dcm2quat(dcm)
    tr = trace(dcm);

    p1 = 1 + tr;
    p2 = 1 + 2 * dcm(1, 1) - tr;
    p3 = 1 + 2 * dcm(2, 2) - tr;
    p4 = 1 + 2 * dcm(3, 3) - tr;
    maxp = max([p1, p2, p3, p4]);

    if maxp == p1
        q1 = 0.5 * sqrt(p1);
        q2 = (dcm(3, 2) - dcm(2, 3)) / 4 / q1;
        q3 = (dcm(1, 3) - dcm(3, 1)) / 4 / q1;
        q4 = (dcm(2, 1) - dcm(1, 2)) / 4 / q1;
    elseif maxp == p2
        q2 = 0.5 * sqrt(p2);
        q3 = (dcm(2, 1) + dcm(1, 2)) / 4 / q2;
        q4 = (dcm(1, 3) + dcm(3, 1)) / 4 / q2;
        q1 = (dcm(3, 2) - dcm(2, 3)) / 4 / q2;
    elseif maxp == p3
        q3 = 0.5 * sqrt(p3);
        q4 = (dcm(3, 2) + dcm(2, 3)) / 4 / q3;
        q1 = (dcm(1, 3) - dcm(3, 1)) / 4 / q3;
        q2 = (dcm(1, 2) + dcm(2, 1)) / 4 / q3;
    elseif maxp == p4
        q4 = 0.5 * sqrt(p4);
        q1 = (dcm(2, 1) - dcm(1, 2)) / 4 / q4;
        q2 = (dcm(1, 3) + dcm(3, 1)) / 4 / q4;
        q3 = (dcm(3, 2) + dcm(2, 3)) / 4 / q4;
    end

    quat = [q1; q2; q3; q4];

    if q1 < 0
        quat = -quat;
    end

end
