% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2025.3.6
% -------------------------------------------------------------------------

function testRotationFunc(rotvec)

    test_pass = true;

    % rotvec -> quat -> rotvec
    quat1 = rotvec2quat(rotvec);
    rotvec1 = quat2rotvec(quat1);

    if ~(norm(rotvec - rotvec1) < 1e-8 || abs(norm(rotvec - rotvec1) - 2 * pi) < 1e-8)
        disp('test quat <--> rotvec failed!');
        test_pass = false;
    end

    % quat -> dcm -> quat
    dcm1 = quat2dcm(quat1);
    quat2 = dcm2quat(dcm1);

    if ~(norm(quat1 - quat2) < 1e-8)
        disp('test quat <--> dcm failed!');
        test_pass = false;
    end

    % dcm -> euler -> dcm
    euler = dcm2euler(dcm1);

    if isnan(euler(1))
        disp('Singular euler angle! Do NOT test euler2dcm');
    else
        dcm2 = euler2dcm(euler);

        if ~(norm(dcm1 - dcm2) < 1e-8)
            disp('test euler <--> dcm failed!');
            test_pass = false;
        end

    end

    if test_pass
        disp(['input rotvec:  ', num2str(rotvec(1)), '，', num2str(rotvec(2)), '，', num2str(rotvec(3)), '，', ' test pass!']);
    else
        disp('testRotation failed!');
    end

end

function testQuat(quat1, quat2, quat3)

    test_pass = true;

    %% normalized test
    quat1 = quatNormalized(quat1);
    quat1_norm = quatNormalized(quat1);

    if ~(norm(quat1) - 1 < 1e-8 && abs(norm(quat1_norm) - 1) < 1e-8 && norm(quat1 - quat1_norm) < 1e-8)
        disp('test quatNormalized failed!');
        test_pass = false;
    end

    %% quatProd test
    prod1 = quatProd(quat1, quatProd(quat2, quat3));
    prod2 = quatProd(quatProd(quat1, quat2), quat3);

    if ~(norm(prod1 - prod2) < 1e-8)
        disp('test quatProd failed!');
        test_pass = false;
    end

    %% quatInv test
    quat1_inv = quatInv(quat1);
    inv_test1 = quatProd(quat1, quat1_inv);
    inv_test2 = quatProd(quat1_inv, quat1);

    if ~(norm(inv_test1 - [1; 0; 0; 0]) < 1e-8 && norm(inv_test2 - [1; 0; 0; 0]) < 1e-8)
        disp('test quatInv failed!');
        test_pass = false;
    end

    if test_pass
        disp('testQuat passed!');
    else
        disp('testQuat failed!');
    end

end

function testQne2BL(lat, lon)
    qne = bl2qne(lat, lon);
    [lat2, lon2] = qne2bl(qne);

    if ~(norm(lat - lat2) < 1e-14 && norm(lon - lon2) < 1e-14)
        disp('test qne2BL failed!');
    else
        disp('test qne2BL passed!');
    end

end

clc; clear; close all;
%% test quat
disp('-------------------------test quaternion--------------------------');
% 生成随机旋转角度
for i = 1:10
    rng('shuffle');
    quat1 = randn(4, 1);
    quat2 = randn(4, 1);
    quat3 = randn(4, 1);
    testQuat(quat1, quat2, quat3);
end

%% test quat2bl
disp('-------------------------test quaternion--------------------------');
% 生成随机旋转角度
for i = 1:10
    rng('shuffle');
    lat = (rand() * 2 - 1) * pi / 2;
    lon = (rand() * 2 - 1) * pi;
    testQne2BL(lat, lon);
end

%% test rotation

disp('-------------------------test rotation--------------------------');
% 特殊旋转
rotvec0 = [0; 0; 0];
rotvec1 = [0; 0; pi];
rotvec2 = [0; 0; pi / 2];
rotvec3 = [0; 0; pi * 2];
rotvec4 = [0; pi / 4; 0];
rotvec5 = [pi / 4; 0; 0];
rotvec6 = [pi / 4; pi / 4; 0];
rotvec7 = [0; pi / 2; 0];
rotvec8 = [pi / 2; 0; 0];
rotvec9 = [pi; 0; 0];

testRotationFunc(rotvec0);
testRotationFunc(rotvec1);
testRotationFunc(rotvec2);
testRotationFunc(rotvec3);
testRotationFunc(rotvec4);
testRotationFunc(rotvec5);
testRotationFunc(rotvec6);
testRotationFunc(rotvec7);
testRotationFunc(rotvec8);
testRotationFunc(rotvec9);

% 生成随机旋转角度
for i = 1:10
    rng('shuffle');
    axis = randn(3, 1);
    axis = axis / norm(axis);
    angle = rand() * 2 * pi;
    rotvec = angle * axis;
    testRotationFunc(rotvec);
end

disp('-------------------------test finishd!--------------------------');
