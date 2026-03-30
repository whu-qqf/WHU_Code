function [kf, navstate] = ErrorFeedback(kf, navstate)

    % position and velocity
    DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
    DR_inv = inv(DR);
    navstate.pos = navstate.pos - DR_inv * kf.x(1:3, 1);
    navstate.vel = navstate.vel - kf.x(4:6, 1);

    % attitude
    qpn = rotvec2quat(kf.x(7:9, 1));
    navstate.qbn = quatProd(qpn, navstate.qbn);
    navstate.cbn = quat2dcm(navstate.qbn);
    navstate.att = dcm2euler(navstate.cbn);

    % imu error
    navstate.gyrbias = navstate.gyrbias + kf.x(10:12, 1);
    navstate.accbias = navstate.accbias + kf.x(13:15, 1);
    navstate.gyrscale = navstate.gyrscale + kf.x(16:18, 1);
    navstate.accscale = navstate.accscale + kf.x(19:21, 1);

    %navstate.odoscale = navstate.odoscale + kf.x(22,1);

    % update some parameters
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    % reset state vector
    kf.x = zeros(kf.RANK, 1);
end