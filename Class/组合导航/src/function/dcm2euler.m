% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Qijin Chen
% Contact : chenqijin@whu.edu.cn
%    Date : Nov. 2019;
% -------------------------------------------------------------------------
% dcm2euler Convert from direction cosine matrix to Euler angles (3-2-1) -NED
%        system
%	[roll, pitch, heading] = dcm2euler(dcm)
%
% INPUTS:
%   dcm = the direction cosine matrix, 3-by-3
% OUTPUTS:
%	roll = roll angle in radians
%	pitch = pitch angle in radians
%	heading = heading angle radians
%
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation,"
%            PhD Thesis, Deparment of Geomatics Engineering, 2005. pp.17-18
% -------------------------------------------------------------------------
function att = dcm2euler(dcm)

    pitch = atan(-dcm(3,1)/sqrt(dcm(3,2)^2 + dcm(3,3)^2));

    if dcm(3,1) <= -0.999
        roll = NaN;
        heading = atan2((dcm(2,3)-dcm(1,2)),(dcm(1,3)+dcm(2,2)));

    elseif dcm(3,1) >= 0.999
        roll = NaN;
        heading = pi + atan2((dcm(2,3)+dcm(1,2)),(dcm(1,3)-dcm(2,2)));

    else
        roll = atan2(dcm(3,2), dcm(3,3));
        heading = atan2(dcm(2,1), dcm(1,1));
    end

    att = [roll; pitch; heading];
end