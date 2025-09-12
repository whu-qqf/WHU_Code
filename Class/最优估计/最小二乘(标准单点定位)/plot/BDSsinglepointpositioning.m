clc;clear; 
dataxyz=importdata("xyz.txt");    %导入数据
dataxyz2=dataxyz.data;    %提取数据
%接收机地心地固坐标系坐标系坐标
x=dataxyz2(:,1);
y=dataxyz2(:,2);
z=dataxyz2(:,3);
  
epoch=dataxyz2(:,5);    %历元
sigma=dataxyz2(:,6);    %验后单位权中误差

clear("dataxyz2");
clear("dataxyz");
%xyz的数字特征
meanx=mean(x);
meany=mean(y);
meanz=mean(z);
stdx=std(x);
stdy=std(y);
stdz=std(z);

dataenu=load("enu.txt");    %导入enu坐标
e=dataenu(:,3);
n=dataenu(:,4);
u=dataenu(:,5);
clear("dataenu");
%%
%画e方向上的时间序列图
figure('Position', [50, 50, 1200, 700]);
subplot(3,1,1);
plot(epoch,e);
title("东(E)方向坐标");
grid on;
axis([0 3000 -2 2]);
xlabel("历元数");
xlim([0,3000]);
ylabel("E(m)");
meane=mean(e);
stde=std(e);
yline(meane, 'r-', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');    %标注均值线
sum=0;
for i=1:length(e)
    sum=sum+e(i)*e(i);
end
rms_e=sqrt(sum/length(e));    %e方向上的均方根值
clear("sum");
legend("E(m)","均值");

%画n方向上的时间序列图
subplot(3,1,2);
plot(epoch,n,'m');
grid on;
title("北(N)方向坐标");
axis([0 3000 -2 2]);
xlabel("历元数");
ylabel("N(m)");
meann=mean(n);
stdn=std(n);
yline(meann, 'g-', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');    %标注均值线
sum=0;
for i=1:length(n)
    sum=sum+n(i)*n(i);
end
rms_n=sqrt(sum/length(n));     %n方向上的均方根值
legend("N(m)","均值");

%画u方向上的时间序列图
subplot(3,1,3);
plot(epoch,u,'k');
title("天(U)方向坐标");
grid on;
axis([0 3000 -2.5 3]);
xlabel("历元数");
ylabel("U(m)");
meanu=mean(u);
stdu=std(u);
yline(meanu, 'b-', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');    %标注均值线
sum=0;
for i=1:length(u)
    sum=sum+u(i)*u(i);
end
rms_u=sqrt(sum/length(u));      %u方向上的均方根值
legend("U(m)","均值");
%%
 %enu坐标系下解算得到的接收机三维散点图
figure;    
scatter3(e,n,u,10,'filled');   
hold on;
plot3([0 1.5], [0 0], [0 0], 'r', 'LineWidth', 2); % X轴 
plot3([0 0], [0 1], [0 0], 'g', 'LineWidth', 2); % Y轴 
plot3([0 0], [0 0], [0 3], 'b', 'LineWidth', 2); %Z周
view(3);
legend("Point","E轴","N轴","U轴");
grid on;
%%
%ENU各方向上的RMS柱状图
figure;
value=[rms_e rms_n rms_u];    %值 
categories={'E','N','U'};    %分类
bar(value,'FaceColor','b','EdgeColor','k','LineWidth',1.5);
title("ENU各方向上RMS");
xlabel('ENU方向');
ylabel('RMS');
set(gca,'XTick',1:length(categories),'XTickLabel',categories);
for i = 1:length(value) 
    text(i, value(i), num2str(value(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
legend("RMS");
grid on;
clear("categories");
clear("value");

%各历元解算值到真值的坐标距离图
for i=1:length(e)
    distance(i)=sqrt(e(i)*e(i)+n(i)*n(i)+u(i)*u(i));
end
figure('Position', [50, 50, 800, 500]);
plot(epoch,distance);
title("各历元解算点到参考真值点距离");
xlabel("历元数");
ylabel("距离(m)");
meand=mean(distance);
yline(meand, 'k--', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');
legend("距离","均值");

%各历元的验后单位权中误差图
figure('Position', [50, 50, 800, 500]);
plot(epoch,sigma);
title("各历元验后单位权中误差");
xlabel("历元数");
ylabel("验后单位权中误差");
legend("验后单位权中误差");


