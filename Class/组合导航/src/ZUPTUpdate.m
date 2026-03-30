function kf = ZUPTUpdate(navstate, laststate, kf, thisimu,is_zerovelocity,dt)
    param = Param();
    if is_zerovelocity==true
        %零速度修正
        Z=navstate.vel-[0;0;0];
        H = zeros(3,kf.RANK);
        H(1:3,4:6)=eye(3);
        R = diag([1;1;1]);
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
        %零角速度修正
        wib_b = thisimu(2:4, 1) / dt;
        wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
        wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
                -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
                -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
        win_n = wie_n + wen_n;
        wnb_b = wib_b - navstate.cbn' * win_n;
        wnb_b_zero = [0;0;0];
        Z = wnb_b-wnb_b_zero;
        H = zeros(3,kf.RANK);
        H(1:3,10:12)=eye(3);
        H(1:3,16:18)=diag(wib_b);
        R = diag([1;1;1]);
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
        %位置锁定
        % DR = diag([laststate.Rm + laststate.pos(3), (laststate.Rn + laststate.pos(3))*cos(laststate.pos(1)), -1]);
        % Z = DR*(navstate.pos - laststate.pos);
        % H = zeros(3, kf.RANK);
        % H(1:3, 1:3) = eye(3);
        % K = kf.P * H' / (H * kf.P * H' + R);
        % kf.x = kf.x + K*(Z - H*kf.x);
        % kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end
end