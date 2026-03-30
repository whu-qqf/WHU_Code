% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2024.12.24
% -------------------------------------------------------------------------
function mat = skew(vec)
    mat = [0, -vec(3), vec(2); 
           vec(3), 0, -vec(1); 
           -vec(2), vec(1), 0];
end