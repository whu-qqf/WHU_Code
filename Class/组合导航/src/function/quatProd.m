% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Qijin Chen
% Contact : chenqijin@whu.edu.cn
%    Date : Nov. 2019;
% -------------------------------------------------------------------------
% QUATPROD Quaternion product
%	q = quatprod(q1, q2)
%
% INPUTS:
%	q1, q2 = input quaternion
% OUTPUTS:
%   q = output quaternion
% 
% Reference: D. H. Titterton and J. L. Weston, Strapdown Inertial Navigation Technology, 2nd ed. 
%            Stevenage, U.K: IET, 2004. pp.43
% -------------------------------------------------------------------------
function q = quatProd(q1, q2)
    q = [ q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3) - q1(4) * q2(4);
          q1(1) * q2(2) + q1(2) * q2(1) + q1(3) * q2(4) - q1(4) * q2(3);
          q1(1) * q2(3) + q1(3) * q2(1) + q1(4) * q2(2) - q1(2) * q2(4);
          q1(1) * q2(4) + q1(4) * q2(1) + q1(2) * q2(3) - q1(3) * q2(2) ];
    
    if q(1) < 0
          q = -q;
    end
    q = quatNormalized(q);
end