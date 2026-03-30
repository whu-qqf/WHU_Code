% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function navstate = InsMech(laststate, lastimu, thisimu)

    param = Param();

    % copy last navstate
    lastpos = laststate.pos;
    lastvel = laststate.vel;
    lastqbn = laststate.qbn;
    lastcbn = laststate.cbn;
    gravity = laststate.gravity;

    % copy imu data
    last_dtheta = lastimu(2:4, 1);
    last_dvel = lastimu(5:7, 1);
    this_dtheta = thisimu(2:4, 1);
    this_dvel = thisimu(5:7, 1);
    this_dt = thisimu(1, 1) - lastimu(1, 1);


    %% velocity update
    % geometric parameters
    rm = laststate.Rm;
    rn = laststate.Rn;
    wie_n = [param.WGS84_WIE * cos(lastpos(1)); 0; -param.WGS84_WIE * sin(lastpos(1))];
    wen_n = [lastvel(2) / (rn + lastpos(3)); 
            -lastvel(1) / (rm + lastpos(3)); 
            -lastvel(2) * tan(lastpos(1)) / (rn + lastpos(3))];

    % rotational and sculling motion
    % 旋转效应和双子样划桨效应
    temp1 = cross(this_dtheta, this_dvel) / 2;
    temp2 = cross(last_dtheta, this_dvel) / 12;
    temp3 = cross(last_dvel, this_dtheta) / 12;

    % velocity increment due to the specific force, b系比例积分项
    d_vfb = this_dvel + temp1 + temp2 + temp3;
    % projected to the n-frame, n系比例积分项
    temp1 = (wie_n + wen_n) * this_dt / 2;
    cnn = eye(3) - skew(temp1);
    d_vfn = cnn * lastcbn * d_vfb;

    % velocity increment due to the gravity and Coriolis force
    % 重力/哥氏积分项
    gl = [0; 0; gravity];
    d_vgn = (gl - cross(2 * wie_n + wen_n, lastvel)) * this_dt;

    % velocity at k-1/2, 中间时刻速度
    midvel = lastvel + (d_vfn + d_vgn) / 2;

    % position at k-1/2, 外推中间时刻的位置
    temp = (wie_n + wen_n) * this_dt / 2;
    qnn = rotvec2quat(temp);
    temp = [0; 0; -param.WGS84_WIE * this_dt / 2];
    qee = rotvec2quat(temp);
    qne = bl2qne(lastpos(1), lastpos(2));
    qne = quatProd(quatProd(qee, qne), qnn);
    [midlat, midlon] = qne2bl(qne);
    midheight = lastpos(3) - midvel(3) * this_dt / 2;
    midpos = [midlat; midlon; midheight];

    % geometric parameters at k-1/2, 重新计算中间时刻的rmrn, wie_e, wen_n
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); -midvel(1) / (rm + midpos(3)); -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];

    % recompute d_vfn, 重新计算n系下平均比力积分项
    temp = (wie_n + wen_n) * this_dt / 2;
    cnn = eye(3) - skew(temp);
    d_vfn = cnn * lastcbn * d_vfb;

    % recompute d_vgn, 重新计算重力、哥式积分项
    gravity = getGravity(midpos);
    gl = [0; 0; gravity];
    d_vgn = (gl - cross(2 * wie_n + wen_n, midvel)) * this_dt;

    % velocity update finish, 速度更新完成
    thisvel = lastvel + d_vfn + d_vgn;


    %% position update
    % recompute velocity and position at k-1/2, 重新计算中间时刻的速度和位置
    midvel = (lastvel + thisvel) / 2;
    DRi = diag([1 / (rm + lastpos(3)), 1 / ((rn + lastpos(3)) * cos(lastpos(1))), -1]);
    midpos = lastpos + DRi * midvel * this_dt / 2;

    % recompute geometric parameters, 重新计算中间时刻地理参数
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); -midvel(1) / (rm + midpos(3)); -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];

    % recompute n-frame rotation vector (n(k) with respect to n(k-1)-frame)
    % 重新计算k时刻到k-1时刻 n系旋转矢量
    temp = (wie_n + wen_n) * this_dt;
    qnn = rotvec2quat(temp);
    % e-frame rotation vector, e系转动等效旋转矢量 (k-1时刻k时刻，所以取负号)
    temp = [0; 0; -param.WGS84_WIE * this_dt];
    qee = rotvec2quat(temp);

    % position update finish, 位置更新完成
    qne = bl2qne(lastpos(1), lastpos(2));
    qne = quatProd(quatProd(qee, qne), qnn);
    [lat, lon] = qne2bl(qne);
    height = lastpos(3) - midvel(3) * this_dt;
    thispos = [lat; lon; height];


    %% attitude update
    % recompute velocity and position at k-1/2, 重新计算中间时刻的速度和位置
    midvel = (lastvel + thisvel) / 2;
    last_qne = bl2qne(lastpos(1), lastpos(2));
    this_qne = bl2qne(thispos(1), thispos(2));
    delta_qnn = quatProd(quatInv(this_qne), last_qne);
    temp = quat2rotvec(delta_qnn);
    qne_mid = quatProd(last_qne, rotvec2quat(temp / 2));
    [midlat, matlon] = qne2bl(qne_mid);
    midheight = (lastpos(3) + thispos(3)) / 2;
    midpos = [midlat; matlon; midheight];

    % recompute geometric paramters, 重新计算中间时刻地理参数
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); 
            -midvel(1) / (rm + midpos(3)); 
            -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];

    % recompute n-frame rotation vector, 计算n系的旋转四元数 k-1时刻到k时刻变换
    temp = -(wie_n + wen_n) * this_dt;
    qnn = rotvec2quat(temp);

    % b-frame rotation vector, 计算b系旋转四元数 补偿二阶圆锥误差
    temp = this_dtheta + cross(last_dtheta, this_dtheta) / 12;
    qbb = rotvec2quat(temp);

    % attitude update finish, 姿态更新完成
    thisqbn = quatProd(quatProd(qnn, lastqbn), qbb);
    thiscbn = quat2dcm(thisqbn);
    thiseuler = dcm2euler(thiscbn);
    
    
    %% update new navstate
    % copy current navstate
    navstate.time = thisimu(1, 1);
    navstate.pos = thispos;
    navstate.vel = thisvel;
    navstate.qbn = thisqbn;
    navstate.cbn = thiscbn;
    navstate.att = thiseuler;
    
    % recompute geometric paramters
    [navstate.Rm, navstate.Rn] = getRmRn(thispos(1), param);
    navstate.gravity = getGravity(thispos);

    % copy IMU error (no update)
    navstate.gyrbias = laststate.gyrbias;
    navstate.accbias = laststate.accbias;
    navstate.gyrscale = laststate.gyrscale;
    navstate.accscale = laststate.accscale;
    navstate.odoscale = laststate.odoscale;
end