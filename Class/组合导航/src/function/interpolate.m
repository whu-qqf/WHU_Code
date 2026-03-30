% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
% -------------------------------------------------------------------------
% interpolate: interpolate IMU data to given time;
% INPUTS: lastimu = last IMU data [time; gyro(3*1); acc(3*1)];
%         thisimu = this IMU data [time; gyro(3*1); acc(3*1)];
%         intertime = interpolation time;
% OUTPUTS: firstimu = first interpolated IMU data; (at intertime)
%          secondimu = second interpolated IMU data; (at time of thisimu)
% -------------------------------------------------------------------------
function [firstimu, secondimu] = interpolate(lastimu, thisimu, intertime)
    firstimu = zeros(7, 1);
    secondimu = zeros(7, 1);
    if (intertime >= lastimu(1, 1) && intertime <= thisimu(1, 1))
        lambda = (intertime - lastimu(1, 1)) / (thisimu(1, 1) - lastimu(1, 1));
        firstimu(1, 1) = intertime;
        firstimu(2:7, 1) = thisimu(2:7, 1) * lambda;
        secondimu(1, 1) = thisimu(1, 1);
        secondimu(2:7, 1) = thisimu(2:7, 1) * (1 - lambda);
    end
end