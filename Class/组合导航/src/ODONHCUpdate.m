% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
% -------------------------------------------------------------------------

function kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();

    %% measurement innovation
    wib_b = thisimu(2:4, 1) / dt;
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;
    wnb_b = wib_b - navstate.cbn' * win_n;

    vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel + skew(wnb_b) * cfg.odolever);
    Z = vel_pre - odonhc_vel;

    %% measurement equation and noise

    % TODO: add measurement equation and noise matrix here!!


    %% update
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K*(Z - H*kf.x);
    kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

end