% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
% -------------------------------------------------------------------------

% importdata navresult
navpath = "dataset_qqf/REF_NAV.nav";
navdata = importdata(navpath);

% velocity
figure()
plot(navdata(:, 2), navdata(:, 6:8));
title('Velocity');
legend('North', 'East', 'Down');
xlabel('Time[s]');
ylabel('Vel[m/s]');
grid("on");

% attitude
figure()
plot(navdata(:, 2), navdata(:, 9:11));
title('Attitude');
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time[s]');
ylabel('Att[deg]');
grid("on");

% position
param = Param();
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * param.D2R;
blh(:, 2) = blh(:, 2) * param.D2R;
first_blh = blh(1, 1:3);

[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(2);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end

%% plane position
figure()
plot(pos(:, 2), pos(:, 1));
title('Position');
xlabel('East[m]');
ylabel('North[m]');
grid("on");

%% height
figure()
plot(navdata(:, 2), navdata(:, 5));
title('Height');
xlabel('Time[s]');
ylabel('Height[m]');
grid("on");





