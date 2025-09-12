
obs=load("observation.txt");
%%EKF
data=load("扩展kalman滤波.txt");
figure;    %%轨迹图
plot(data(:,2),data(:,3),'-o', ...
    'MarkerSize',3);
hold on;
plot(obs(:,1).*sin(obs(:,2)),obs(:,1).*cos(obs(:,2)),'-*', ...
    'Color','r')
title("观测值与EKF估计轨迹对比");
legend('估计轨迹','观测值');
xlabel('X(m)');
ylabel('Y(m)');

figure;    %%x和y滤波方差图
plot(data(:,1),data(:,4),'-o', ...
    'MarkerFaceColor', 'b', ...
    'MarkerEdgeColor', 'b');
hold on;
plot(data(:,1),data(:,5),'-*');
title('x和y的滤波方差');
legend('X的方差','Y的方差');
ylabel('m^2');
xlabel('t(s)');

figure;    %%新息图
subplot(2,1,1);
plot(data(2:end,1),data(2:end,6),'-o', ...
    'MarkerSize',3);
legend('r的新息');
xlabel('t(s)');
ylabel('(m)');
subplot(2,1,2);
plot(data(2:end,1),data(2:end,7),'-*', ...
    'MarkerSize',3);
legend('α的新息');
xlabel('t(s)');
ylabel('(rad)');

figure;    %%增益图
subplot(2,2,1);
plot(data(2:end,1),data(2:end,8),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('X的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,2);
plot(data(2:end,1),data(2:end,9),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('V_x的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;
subplot(2,2,3);
plot(data(2:end,1),data(2:end,10),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('Y的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,4);
plot(data(2:end,1),data(2:end,11),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('V_y的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;

%%
% 观测值逐次更新的EKF
data_1=load("观测值逐次更新的扩展kalman滤波.txt");

figure;    %%轨迹图
plot(data_1(:,2),data_1(:,3),'-o', ...
    'MarkerSize',3);
hold on;
plot(obs(:,1).*sin(obs(:,2)),obs(:,1).*cos(obs(:,2)),'-*', ...
    'Color','r')
title("观测值与观测值逐次更新的EKF估计轨迹对比");
legend('估计轨迹','观测值');
xlabel('X(m)');
ylabel('Y(m)');

figure;    %%滤波方差图
plot(data_1(:,1),data_1(:,4),'-o', ...
    'MarkerFaceColor', 'b', ...
    'MarkerEdgeColor', 'b');
hold on;
plot(data_1(:,1),data_1(:,5),'-*');
title('x和y的滤波方差');
legend('X的方差','Y的方差');
ylabel('m^2');
xlabel('t(s)');

figure;     %%新息图
subplot(2,1,1);
plot(data_1(2:end,1),data_1(2:end,6),'-o', ...
    'MarkerSize',3);
legend('r的新息');
xlabel('t(s)');
ylabel('(m)');
subplot(2,1,2);
plot(data_1(2:end,1),data_1(2:end,7),'-*', ...
    'MarkerSize',3);
legend('α的新息');
xlabel('t(s)');
ylabel('(rad)');

figure;    %%r增益图
subplot(2,2,1);
plot(data_1(2:end,1),data_1(2:end,8),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('r对X的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,2);
plot(data_1(2:end,1),data_1(2:end,9),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('r对V_x的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;
subplot(2,2,3);
plot(data_1(2:end,1),data_1(2:end,10),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('r对Y的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,4);
plot(data_1(2:end,1),data_1(2:end,11),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('r对V_y的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;

figure;    %%α增益图
subplot(2,2,1);
plot(data_1(2:end,1),data_1(2:end,12),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('α对X的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,2);
plot(data_1(2:end,1),data_1(2:end,13),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('α对V_x的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;
subplot(2,2,3);
plot(data_1(2:end,1),data_1(2:end,14),'-o', ...
    'Color','k', ...
    'MarkerFaceColor', 'k', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('α对Y的增益');
xlabel('t(s)');
ylabel('(m)');
grid on;
subplot(2,2,4);
plot(data_1(2:end,1),data_1(2:end,15),'-*', ...
    'Color','k', ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'MarkerSize',2);
axis([0, 10, -5, 5]);
legend('α对V_y的增益');
xlabel('t(s)');
ylabel('(m/s)');
grid on;
%%
figure;
plot(data(:,2),data(:,3));
hold on;
plot(data_1(:,2),data_1(:,3));