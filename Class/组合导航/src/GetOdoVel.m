% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
% -------------------------------------------------------------------------

% average odometer data to decrease quantization noise
% 对一段ODO数据平均，获取某一时刻ODO速度，可减小量化噪声
% 相当于匀变速假设，有一定精度损失
function [odovel, valid] = GetOdoVel(odoraw, time)

    odovel = 0;
    valid = false;

    if (size(odoraw, 1) < 5)
        disp('WARN: too little data to get odovel!');
    else

        firstvel = 0;
        secondvel = 0;
        firsttime = 0;
        secondtime = 0;
        firstindex = 0;
        secondindex = 0;
    
        for i = 1:size(odoraw, 1)
            if odoraw(i, 1) <= time
                firstvel = (firstvel * firstindex + odoraw(i, 2)) / (firstindex + 1);
                firsttime = (firsttime * firstindex + odoraw(i, 1)) / (firstindex + 1);
                firstindex = firstindex + 1;
            else
                secondvel = (secondvel * secondindex + odoraw(i, 2)) / (secondindex + 1);
                secondtime = (secondtime * secondindex + odoraw(i, 1)) / (secondindex + 1);
                secondindex = secondindex + 1;
            end
        end
    
        odovel = firstvel + (time - firsttime) / (secondtime - firsttime) * (secondvel - firstvel);
        valid = true;
    end
end