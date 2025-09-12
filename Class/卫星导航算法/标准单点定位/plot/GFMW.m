function utc_time = gps2utc(gps_week, seconds_of_week)
    gps_start = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
    total_seconds = gps_week * 604800 + seconds_of_week - 18;
    utc_time = gps_start + seconds(total_seconds);
end
%%



% 读取并处理Combo.txt数据文件，绘制GPS和BDS卫星的GF/MW组合观测值时序图
clear; clc; close all;

% 初始化数据结构
data = struct('week', {}, 'sow', {}, 'numSats', {}, 'satellites', {});
satTypes = struct('system', {}, 'prn', {}, 'type', {});  % 存储卫星类型信息

% 打开文件
fid = fopen('Combo.txt', 'r');
if fid == -1
    error('无法打开文件 Combo.txt');
end

% 预定义卫星类型 (根据常见北斗卫星分配规则)
% GEO: PRN 1-10 (示例: C01-C10), IGSO: 11-20, MEO: 21-40
% 注意: 实际文件中的PRN可能不同，这里根据示例数据自定义类型
bdsTypes = containers.Map();
bdsTypes('C3') = 'GEO';
bdsTypes('C2') = 'GEO';
bdsTypes('C7') = 'IGSO';
bdsTypes('C8') = 'IGSO';
bdsTypes('C36') = 'MEO';
bdsTypes('C30')  = 'MEO';

% 选择目标卫星 (5颗GPS + 6颗BDS)
targetGPSSats = {'G3', 'G4', 'G9', 'G27', 'G30'};
targetBDSSats = {'C3', 'C2', 'C7', 'C8', 'C36', 'C30'};

% 读取文件
epochCount = 0;
while ~feof(fid)
    % 读取历元头信息
    header = fgetl(fid);
    if ~ischar(header), break; end
    headerData = sscanf(header, '%f %f %f');
    
    epochCount = epochCount + 1;
    data(epochCount).week = headerData(1);
    data(epochCount).sow = headerData(2);
    data(epochCount).numSats = headerData(3);
    data(epochCount).satellites = struct('id', {}, 'gf', {}, 'mw', {});
    
    % 读取卫星数据
    for i = 1:data(epochCount).numSats
        satLine = fgetl(fid);
        satData = textscan(satLine, '%s %f %f');
        
        satID = satData{1}{1};
        satGF = satData{2};
        satMW = satData{3};
        
        % 存储卫星数据
        data(epochCount).satellites(i).id = satID;
        data(epochCount).satellites(i).gf = satGF;
        data(epochCount).satellites(i).mw = satMW;
    end
end
fclose(fid);

for i=1:length(data)
    time(i)=gps2utc(data(i).week,data(i).sow)+hours(8);
end
% 提取时间序列
% time = [data.sow]';  % 周内秒时间序列

% 初始化观测值矩阵 (NaN表示缺失数据)
gfGPS = nan(length(time), length(targetGPSSats));
mwGPS = nan(length(time), length(targetGPSSats));
gfBDS = nan(length(time), length(targetBDSSats));
mwBDS = nan(length(time), length(targetBDSSats));

% 填充观测值矩阵
for epoch = 1:length(data)
    for satIdx = 1:length(data(epoch).satellites)
        satID = data(epoch).satellites(satIdx).id;
        
        % 检查GPS卫星
        gpsIdx = find(strcmp(targetGPSSats, satID));
        if ~isempty(gpsIdx)
            gfGPS(epoch, gpsIdx) = data(epoch).satellites(satIdx).gf;
            mwGPS(epoch, gpsIdx) = data(epoch).satellites(satIdx).mw;
        end
        
        % 检查BDS卫星
        bdsIdx = find(strcmp(targetBDSSats, satID));
        if ~isempty(bdsIdx)
            gfBDS(epoch, bdsIdx) = data(epoch).satellites(satIdx).gf;
            mwBDS(epoch, bdsIdx) = data(epoch).satellites(satIdx).mw;
        end
    end
end


%% 绘制GF组合时序图
figure;

% GPS卫星GF值
subplot(2, 1, 1);
hold on;
gpsColors = lines(length(targetGPSSats));
gpsHandles = gobjects(1, length(targetGPSSats));

for i = 1:length(targetGPSSats)
    plot(time, gfGPS(:, i), 'LineWidth', 1.8, ...
        'Color', gpsColors(i, :), ...
        'DisplayName', ['GPS ' targetGPSSats{i}]);
end

title('GF组合观测值时序图', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('UTC+8h', 'FontSize', 12);
ylabel('GF', 'FontSize', 12);
grid on;
legend('show', 'Location', 'eastoutside');
set(gca, 'FontSize', 11);

% BDS卫星GF值 (按类型区分线型)
subplot(2, 1, 2);
hold on;
bdsColors = jet(length(targetBDSSats));
bdsHandles = gobjects(1, length(targetBDSSats));

for i = 1:length(targetBDSSats)
    satID = targetBDSSats{i};
    lineStyle = '-';  % 默认实线
    
    % 根据卫星类型设置线型
    switch bdsTypes(satID)
        case 'GEO'
            lineStyle = '-';  % 实线
        case 'IGSO'
            lineStyle = '-'; % 虚线
        case 'MEO'
            lineStyle = '-';  % 点线
    end
    
    plot(time, gfBDS(:, i), lineStyle, 'LineWidth', 1.8, ...
        'Color', bdsColors(i, :), ...
        'DisplayName', [satID ' (' bdsTypes(satID) ')']);
end

title('BDS卫星GF组合观测值', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('UTC+8h', 'FontSize', 12);
ylabel('GF', 'FontSize', 12);
grid on;
legend('show', 'Location', 'eastoutside');
set(gca, 'FontSize', 11);

%% 绘制MW组合时序图
figure;

% GPS卫星MW值
subplot(2, 1, 1);
hold on;
for i = 1:length(targetGPSSats)
    plot(time, mwGPS(:, i), 'LineWidth', 1.8, ...
        'Color', gpsColors(i, :), ...
        'DisplayName', ['GPS ' targetGPSSats{i}]);
end

title('MW组合观测值时序图', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('UTC+8h', 'FontSize', 12);
ylabel('MW', 'FontSize', 12);
grid on;
legend('show', 'Location', 'eastoutside');
set(gca, 'FontSize', 11);

% BDS卫星MW值 (按类型区分线型)
subplot(2, 1, 2);
hold on;
for i = 1:length(targetBDSSats)
    satID = targetBDSSats{i};
    lineStyle = '-';  % 默认实线
    
    % 根据卫星类型设置线型
    switch bdsTypes(satID)
        case 'GEO'
            lineStyle = '-';  % 实线
        case 'IGSO'
            lineStyle = '-'; % 虚线
        case 'MEO'
            lineStyle = '-';  % 点线
    end
    
    plot(time, mwBDS(:, i), lineStyle, 'LineWidth', 1.8, ...
        'Color', bdsColors(i, :), ...
        'DisplayName', [satID ' (' bdsTypes(satID) ')']);
end

title('BDS卫星MW组合观测值', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('UTC+8h', 'FontSize', 12);
ylabel('MW', 'FontSize', 12);
grid on;
legend('show', 'Location', 'eastoutside');
set(gca, 'FontSize', 11);

% 保存高清图片
%saveas(gcf, 'MW_Combination_Plot.png', 'png');