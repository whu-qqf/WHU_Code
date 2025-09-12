clc;clear; 
data1=importdata("C30ClockError.txt");    %导入数据
data2=data1.data;     %获取data1中的数据
clear data1;      %删除data1，释放内存
year=data2(:,1);     %年
month=data2(:,2);    %月
day=data2(:,3);      %日
hour=data2(:,4);     %小时
minute=data2(:,5);    %分钟
second=data2(:,6);    %秒
dt=data2(:,9);      %clock error

%将数据转化为标准时间格式
time=datetime(year,month,day,hour,minute,second);

%清除NaN异常数据
index=[];       %获取异常数据的索引值
for i=1:length(dt)
    if isnan(dt(i))
        index(end+1)=i;
    end
end
dt(index)=[];    %删除
time(index)=[];

%计算均方根值
sum=0;
for i=1:length(dt)
    sum=sum+dt(i)*dt(i);  
end
rms=sqrt(sum/length(dt));

%画未修正的时间序列图
figure;     %创建一个图窗
plot(time,dt);     
%添加注解标签
dim=[0.2 0.6 0.3 0.3];
str=sprintf('均值：%f;中误差：%f;RMS：%f',mean(dt),std(dt),rms);   %注解字符串
annotation('textbox',dim,'String',str,'FitBoxToText','on');   %添加注解标签
clear str;   %删除str，释放内存
[maxValue, maxIndex] = max(dt);      %获得最大值坐标
[minValue, minIndex] = min(dt);      %获得最小值坐标
%为图像增加元素
xlabel("时间");
ylabel("clock error");
title("BDS3 clock error 时间序列图");
hold on;     
plot(time(maxIndex), maxValue, 'r*', 'MarkerSize', 10); % 用红色星号标记最大值 
text(time(maxIndex), maxValue, sprintf('最大值: %.4f', maxValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');  %最大值处标记
hold on;
plot(time(minIndex), minValue, 'b*', 'MarkerSize', 10); % 用蓝色星号标记最小值 
text(time(minIndex), minValue, sprintf('最小值: %.4f', minValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');  %最小值处标记
legend('C30','最大值点','最小值点');      %显示图例


%画频数直方图
figure('Position', [300, 300, 1000, 400]);
subplot(1,2,1);
histogram(dt);
set(gca, 'YScale', 'log');
xlabel("clock error");
ylabel("number");
title("BDS3 clock error 频数直方图");
legend('C30');

subplot(1,2,2);
histogram(dt, 'Normalization', 'probability');
xlabel("clock error");
ylabel("rate");
title("BDS3 clock error 频率直方图");
legend('C30');



%清除大于3被方差的异常数据
index=[];   %获取异常数据的索引值
for i=1:length(dt)
    if abs(dt(i)-mean(dt))>3*std(dt)
        index(end+1)=i;
    end
end
dt(index)=[];    %删除
time(index)=[];

%修正数据后
%计算均方根值
sum=0;
for i=1:length(dt)
    sum=sum+dt(i)*dt(i);
end
rms=sqrt(sum/length(dt));

%画修正后的时间序列图
figure;
plot(time,dt);
%添加注解
dim=[0.2 0.6 0.3 0.3];
str=sprintf('均值：%f;中误差：%f;RMS：%f',mean(dt),std(dt),rms);
annotation('textbox',dim,'String',str,'FitBoxToText','on');
clear str;   %释放内存
[maxValue, maxIndex] = max(dt);    %获取最大值坐标
[minValue, minIndex] = min(dt);    %获取最小值坐标
%增加元素
xlabel("时间");
ylabel("clock error");
title("BDS3 clock error 修正时间序列图");
ylim([-10,10]);       %改变y轴显示范围，使得对比更明显
hold on;     
plot(time(maxIndex), maxValue, 'r*', 'MarkerSize', 10); % 用红色星号标记最大值 
text(time(maxIndex), maxValue, sprintf('最大值: %.4f', maxValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');  %最大值处标记
hold on;
plot(time(minIndex), minValue, 'b*', 'MarkerSize', 10); % 用蓝色星号标记最小值 
text(time(minIndex), minValue, sprintf('最小值: %.4f', minValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');  %最小值处标记
legend('C30','最大值点','最小值点');      %显示图例

%画出修正后的频数直方图
figure('Position', [100, 100, 1000, 400]);
subplot(1,2,1);
histogram(dt,'BinWidth',0.1);
set(gca, 'YScale', 'log');
xlabel("clock error");
ylabel("number");
title("BDS3 clock error 修正频数直方图");
legend('C30');

subplot(1,2,2);
histogram(dt, 'Normalization', 'probability','BinWidth',0.1);
xlabel("clock error");
ylabel("rate");
title("BDS3 clock error 修正频率直方图");
legend('C30');

%计算时间序列的相关系数
sum=0;
for i=1:length(dt)
    sum=sum+(dt(i)-mean(dt))*(dt(i)-mean(dt));
end
V=sum/(length(dt)-1);      %计算分母
P=[];       %存放Δn对应的相关系数

index=[];   %存放Δn
for i=4:192
    index(end+1)=i;
    sum=0;
    for j=1:length(dt)-i
        sum=sum+(dt(j)-mean(dt))*(dt(j+i)-mean(dt));
    end
    sum=sum/(length(dt)-i);       %计算分子
    P(end+1)=sum/V;     %放入
end

%画出相关系数图像
figure;
plot(index,P,'-o');
xlabel("Δn");
ylabel("ρ（Δn）");
title("相关系数");
[maxValue, maxIndex] = max(P);   %存储最大值的坐标信息
[minValue,minIndex]=min(P);    %存储最小值的坐标信息
hold on;     
plot(index(maxIndex), maxValue, 'r*', 'MarkerSize', 10); % 用红色星号标记最大值 
text(index(maxIndex), maxValue, sprintf('Δn=%d  最大值: %.4f',index(maxIndex), maxValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');  %最大值处标记
hold on;
plot(index(minIndex), minValue, 'b*', 'MarkerSize', 10); % 用蓝色星号标记最小值 
text(index(minIndex), minValue, sprintf('Δn=%d  最小值: %.4f',index(minIndex), minValue), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');  %最小值处标记
hold on;
yline(mean(P), 'r--', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');
%plot(index,mean(P),'g');
% 显示图例 
legend('相关系数', '最大值','最小值','平均值线');
clear;



