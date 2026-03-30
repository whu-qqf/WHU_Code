function kf = ODONHCUpdate_qqf(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();
    wib_b = thisimu(2:4, 1) / dt;
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;
    wnb_b = wib_b - navstate.cbn' * win_n;
    vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel + skew(wnb_b) * cfg.odolever);
    Z = vel_pre - odonhc_vel;
    H = zeros(3, kf.RANK);
    H(1:3,4:6) = cfg.cbv *navstate.cbn';
    H(1:3,7:9) = -cfg.cbv*navstate.cbn'*skew(navstate.vel)-cfg.cbv*skew(cfg.odolever)*navstate.cbn'*skew(win_n);
    H(1:3,10:12) = -cfg.cbv*skew(cfg.odolever);
    H(1:3,16:18) = -cfg.cbv*skew(cfg.odolever)*diag(wib_b);

    H(1:3,22)=-odonhc_vel;
    
    R=diag(power(cfg.odonhc_measnoise,2));

    %% measurement innovation
    if cfg.usenhconly==false
        Z_odo = Z(1);
        H_odo = H(1,:);
        R_odo = R(1,1);
        K = kf.P * H_odo' / (H_odo * kf.P * H_odo' + R_odo);
        kf.x = kf.x + K*(Z_odo - H_odo*kf.x);
        kf.P=(eye(kf.RANK) - K*H_odo) * kf.P * (eye(kf.RANK) - K*H_odo)' + K * R_odo * K';
    end

    if cfg.useodoonly==false
        Z_nhc = Z(2:3,1);
        H_nhc = H(2:3,:);
        R_nhc = R(2:3,2:3);
        K = kf.P * H_nhc' / (H_nhc * kf.P * H_nhc' + R_nhc);
        kf.x = kf.x + K*(Z_nhc - H_nhc*kf.x);
        kf.P=(eye(kf.RANK) - K*H_nhc) * kf.P * (eye(kf.RANK) - K*H_nhc)' + K * R_nhc * K';
    end

end