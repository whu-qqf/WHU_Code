% 读取 RINEX 观测文件
obsData = rinexread('E049_3.25o');
%%
gpsVars = obsData.GPS.Properties.VariableNames;
metaCols = {'SatelliteID','EpochFlag','ReceiverClockOffset'};
idx = ~ismember(gpsVars, metaCols);
idx = idx & ~endsWith(gpsVars, '_LLI') & ~endsWith(gpsVars, '_SSI');
mainObsVars = gpsVars(idx);
disp('GPS 主观测列：');
disp(mainObsVars);
tbl = obsData.GPS;
for k = 1:numel(mainObsVars)
    v = mainObsVars{k};
    data = tbl.(v);
    data(data == 0) = NaN;
    tbl.(v) = data;
end
epochCount = height(tbl);
DI_f = zeros(numel(mainObsVars),1);
for k = 1:numel(mainObsVars)
    v = mainObsVars{k};
    DI_f(k) = sum(~isnan(tbl.(v))) / epochCount * 100;
end

for k = 1:numel(mainObsVars)
    fprintf('%s: %.2f%%\n', mainObsVars{k}, DI_f(k));
end
figure;
bar(DI_f, 'FaceColor', [0.2 0.6 0.8]);
set(gca, ...
    'XTick', 1:numel(mainObsVars), ...
    'XTickLabel', mainObsVars, ...
    'XTickLabelRotation', 45, ...
    'FontName', 'SimHei', ...
    'FontSize', 12);
xlabel('频点类型');
ylabel('单频点数据完整率 (%)');
title('GPS系统各频点完整率');
ylim([0 100]);
grid on;
hold on;
for i = 1:numel(DI_f)
    x = i;
    y = DI_f(i);
    text(x, y-1, sprintf('%.2f%%', y), ...  % y+1 让文本稍微高出柱顶
        'HorizontalAlignment','center', ...
        'FontName','SimHei', ...
        'FontSize',10);
end
hold off;
T = height(tbl);
allData = [];           % 存放观测值
allValidFlag = [];      % 每行是否“全频点非NaN”

for i = 1:T
    sats = tbl.SatelliteID(i);    % 当前历元的卫星列表（1×m）
    m = numel(sats);
    K = numel(mainObsVars);
    
    rowBlock = NaN(m, K);         % m×K，每行一个卫星，每列一个频点
    
    for k = 1:K
        obsVec = tbl.(mainObsVars{k})(i);   % 第 k 个频点在当前历元的观测值（m×1）
        rowBlock(:,k) = obsVec;
    end
    allData = [allData; rowBlock];
    allValidFlag = [allValidFlag; all(~isnan(rowBlock), 2)];
end

sumTheo = size(allData, 1);
sumFull = sum(allValidFlag);
DI_s = sumFull / sumTheo * 100;
fprintf('GPS 单系统观测数据完整率 DI_s = %.2f%%\n', DI_s);
%%
clear time_data;
clear obs_data;
for k=1:T
    time_data(k,1)=tbl.Time(k);
    ID(k,1)=tbl.SatelliteID(k);
end
% 将 obs_data 转换为表格并指定列名 (示例列名)

obs_table = array2table(allData, 'VariableNames', ...
    mainObsVars);
id_table=table(ID,'VariableNames',{'ID'});
% 创建包含时间列的完整表格
combined_table = table(time_data, 'VariableNames', {'Time'});
combined_table = [combined_table,id_table, obs_table]; % 水平拼接表格

G05_data=combined_table(combined_table.ID==5,:);
%%
GPS_L1=1575.42e6;
GPS_L2=1227.60e6;
c=299792458;
G05_time=G05_data.Time;
G05_C1C=G05_data.C1C;
G05_C2W=G05_data.C2W;
G05_L1C=G05_data.L1C;
G05_L2W=G05_data.L2W;
lambda1=c/GPS_L1;
lambda2=c/GPS_L2;
G05_MW=(G05_L1C*c-G05_L2W*c)/(GPS_L1-GPS_L2)-(G05_C1C*GPS_L1+GPS_L2*G05_C2W)/(GPS_L1+GPS_L2);
G05_GF=G05_L1C*lambda1-G05_L2W*lambda2;
figure;

subplot(2,1,1);
plot(G05_time,G05_MW,'Color','r');
xlabel('UTC');
ylabel('MW组合观测值(m)');
title('G05卫星MW组合观测值');
grid on;
subplot(2,1,2);
plot(G05_time,G05_GF);
xlabel('UTC');
ylabel('GF组合观测值(m)');
title('G05卫星GF组合观测值');
grid on;
%%
G05_MWP=[];
sigma_2=[];
errorocc=0;
j=0;
for i=1:length(G05_MW)
    j=j+1;
    if i==1
        G05_MWP(i)=G05_MW(i);
        sigma_2(i)=0;
    else
        if j>2 && abs(G05_MW(i)-G05_MWP(i-1))>=4*sqrt(sigma_2(i-1))
            errorocc=[errorocc,i];
            %
            sigma_2(i)=(j-1)*sigma_2(i-1)/j+(G05_MW(i)-G05_MWP(i-1))*(G05_MW(i)-G05_MWP(i-1))/j;
            G05_MWP(i)=G05_MW(i)/j+(j-1)*G05_MWP(i-1)/j;
            a=j*G05_MWP(i)/(j+1)+G05_MW(i+1)/(j+1);

            if abs(a-G05_MWP(i))<4*sqrt(sigma_2(i)) || abs(G05_MW(i+1)-G05_MW(i))>1
                disp('发生粗差');
            end
            if abs(a-G05_MWP(i))>=4*sqrt(sigma_2(i)) && abs(G05_MW(i+1)-G05_MW(i))<=1
                disp('发生周跳');
            end

            %
            j=0;
            G05_MWP(i)=G05_MW(i);
            sigma_2(i)=0;
        else
            sigma_2(i)=(j-1)*sigma_2(i-1)/j+(G05_MW(i)-G05_MWP(i-1))*(G05_MW(i)-G05_MWP(i-1))/j;
            G05_MWP(i)=G05_MW(i)/j+(j-1)*G05_MWP(i-1)/j;
            
        end
    end
end 
figure;
plot(G05_time,G05_MWP);
grid on;
xlabel('UTC');
ylabel('MW组合平滑值(m)');
title('MW平滑值');
%%
% G05_C1C(387)=(G05_C1C(386)+G05_C1C(388))/2;
% G05_C2W(387)=(G05_C2W(386)+G05_C2W(388))/2;
% G05_L1C(387)=(G05_L1C(386)+G05_L1C(388))/2;
% G05_L2W(387)=(G05_L2W(386)+G05_L2W(388))/2;
% G05_C1C(387)=mean(G05_C1C);
% G05_C2W(387)=mean(G05_C2W);
% G05_L1C(387)=mean(G05_L1C);
% G05_L2W(387)=mean(G05_L2W);
% G05_C1C(513)=mean(G05_C1C);
% G05_C2W(513)=mean(G05_C2W);
% G05_L1C(513)=mean(G05_L1C);
% G05_L2W(513)=mean(G05_L2W);
%%
GF_diff=[];
time_diff_seconds = seconds(diff(G05_time)); % 长度N-1
time_mid = G05_time(1:end-1) + seconds(time_diff_seconds / 2); % 关键步骤
for i=1:length(G05_GF)-1
    GF_diff(i)=G05_GF(i+1)-G05_GF(i);
end
figure;
plot(time_mid,GF_diff);
xlabel('UTC');
ylabel('GF一次差(m)');
title('GF一次差时序图');
%%
deltaL=[];
for i=1:length(G05_GF)-1
    deltaL(i)=abs((G05_C1C(i+1)-G05_C1C(i))-(G05_L1C(i+1)*lambda1-G05_L1C(i)*lambda1));
end
A=c*1e-7-12;
B=c*1e-5+12;
C=c*1e-3-12;
plot(time_mid,deltaL);
xlabel('UTC');
ylabel('钟跳探测量(m)');
title('钟跳探测时序图');

%%
MP_1=G05_C1C-(GPS_L1*GPS_L1+GPS_L2*GPS_L2)*G05_L1C*lambda1/(GPS_L1*GPS_L1-GPS_L2*GPS_L2)+2*GPS_L2*GPS_L2*G05_L2W*lambda2/(GPS_L1*GPS_L1-GPS_L2*GPS_L2);
MP_2=G05_C2W-2*GPS_L1*GPS_L1*G05_L1C*lambda1/(GPS_L1*GPS_L1-GPS_L2*GPS_L2)+(GPS_L1*GPS_L1+GPS_L2*GPS_L2)*G05_L2W*lambda2/(GPS_L1*GPS_L1-GPS_L2*GPS_L2);
MP_k=[];
sum1=0;
sum2=0;
for i=1:length(G05_L1C)-49
    sum1=0;
    sum2=0;
    for j=i:i+49
        sum1=sum1+MP_1(j);
    end
    MP_k(i)=MP_1(i)-sum1/50;
end
for i=length(G05_L2W)-48:length(G05_L2W)
    MP_k(i)=MP_1(i)-sum1/50;
end
figure;
plot(G05_time,MP_k);
grid on;
xlabel('UTC');
ylabel('L1多路径误差(m)');
E_m=mean(MP_k);
E_rms=rms(MP_k);
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title('L1多路径时序图');
%%
for i=1:length(G05_L1C)-49
    sum1=0;
    sum2=0;
    for j=i:i+49
        sum1=sum1+MP_2(j);
    end
    MP_k(i)=MP_2(i)-sum1/50;
end
for i=length(G05_L2W)-48:length(G05_L2W)
    MP_k(i)=MP_2(i)-sum1/50;
end
figure;
plot(G05_time,MP_k);
grid on;
xlabel('UTC');
E_m=mean(MP_k);
E_rms=rms(MP_k);
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
ylabel('L2多路径误差(m)');
title('L2多路径时序图');

%%
bdsVars = obsData.BeiDou.Properties.VariableNames;
metaCols = {'SatelliteID','EpochFlag','ReceiverClockOffset'};
idx = ~ismember(bdsVars, metaCols);
% 3. 再去掉所有以 _LLI 或 _SSI 结尾的列
idx = idx & ~endsWith(bdsVars, '_LLI') & ~endsWith(bdsVars, '_SSI');
% 4. 得到主观测列列表
mainObsVars = bdsVars(idx);
% 5. 显示
disp('BDS 主观测列：');
disp(mainObsVars);
tbl = obsData.BeiDou;
% 将所有主观测列里的 0 替换成 NaN
for k = 1:numel(mainObsVars)
    v = mainObsVars{k};
    data = tbl.(v);
    data(data == 0) = NaN;
    tbl.(v) = data;
end
epochCount = height(tbl);
DI_f = zeros(numel(mainObsVars),1);
for k = 1:numel(mainObsVars)
    v = mainObsVars{k};
    DI_f(k) = sum(~isnan(tbl.(v))) / epochCount * 100;
end
for k = 1:numel(mainObsVars)
    fprintf('%s: %.2f%%\n', mainObsVars{k}, DI_f(k));
end
figure;
bar(DI_f, 'FaceColor', [0.2 0.6 0.8]);
set(gca, ...
    'XTick', 1:numel(mainObsVars), ...
    'XTickLabel', mainObsVars, ...
    'XTickLabelRotation', 45, ...
    'FontName', 'SimHei', ...
    'FontSize', 12);
xlabel('频点类型');
ylabel('单频点数据完整率 (%)');
title('BDS系统各频点完整率');
ylim([0 100]);
grid on;

% 手动在每根柱子上方添加数值
hold on;
for i = 1:numel(DI_f)
    x = i;
    y = DI_f(i);
    text(x, y-1, sprintf('%.2f%%', y), ...  % y+1 让文本稍微高出柱顶
        'HorizontalAlignment','center', ...
        'FontName','SimHei', ...
        'FontSize',10);
end
hold off;
%%
T = height(tbl);
allData = [];           % 存放观测值
allValidFlag = [];      % 每行是否“全频点非NaN”

for i = 1:T
    sats = tbl.SatelliteID(i);    % 当前历元的卫星列表（1×m）
    m = numel(sats);
    K = numel(mainObsVars);
    
    rowBlock = NaN(m, K);         % m×K，每行一个卫星，每列一个频点
    
    for k = 1:K
        obsVec = tbl.(mainObsVars{k})(i);   % 第 k 个频点在当前历元的观测值（m×1）
        rowBlock(:,k) = obsVec;
    end
    allData = [allData; rowBlock];
    allValidFlag = [allValidFlag; all(~isnan(rowBlock), 2)];
end
sumTheo = size(allData, 1);
sumFull = sum(allValidFlag);
DI_s = sumFull / sumTheo * 100;
fprintf('BDS 单系统观测数据完整率 DI_s = %.2f%%\n', DI_s);
%%

