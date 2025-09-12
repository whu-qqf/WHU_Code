% 读取数据文件
filename = 'result.txt';
fid = fopen(filename, 'r');
data = textscan(fid, '%d %d %f %s %f %f');
fclose(fid);

% 解析数据列
hours = data{1};
mins = data{2};
secs = data{3};
prn_list = data{4};
MW = data{5};
GF = data{6};

% 转换时间为秒
time_seconds = double(hours)*3600 + double(mins)*60 + secs;
% GPS卫星（5颗）
gps_prns = {'G02', 'G05', 'G13', 'G15', 'G20'};

% 北斗卫星（2GEO + 2IGSO + 2MEO）
bds_geo_prns = {'C01', 'C02'};   % GEO
bds_igso_prns = {'C06', 'C08'};  % IGSO
bds_meo_prns = {'C11', 'C12'};   % MEO

% 合并所有选择的北斗卫星
bds_prns = [bds_geo_prns, bds_igso_prns, bds_meo_prns];
all_target_prns = [gps_prns, bds_prns];
unique_prns = unique(prn_list);
valid_prns = intersect(all_target_prns, unique_prns); % 实际存在的卫星

full_time = (0:30:86370)'; % 24小时=86400秒，最后一个点86370

MW_matrix = nan(length(full_time), length(valid_prns));
GF_matrix = nan(length(full_time), length(valid_prns));

% 填充数据
for i = 1:length(valid_prns)
    prn = valid_prns{i};
    idx = strcmp(prn_list, prn);
    [~, pos] = ismember(time_seconds(idx), full_time);
    MW_matrix(pos, i) = MW(idx);
    GF_matrix(pos, i) = GF(idx);
end
% 定义颜色和线型
colors = lines(6); % 预定义6种颜色
line_styles = {'-', '--', ':'}; % 用于区分北斗类型
figure;
% 提取GPS卫星的索引
gps_idx = find(ismember(valid_prns, gps_prns));

subplot(2,1,1)
hold on
for i = 1:length(gps_idx)
    plot(full_time/3600, MW_matrix(:,gps_idx(i)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', valid_prns{gps_idx(i)})
end
xlabel('时间 (小时)')
ylabel('MW (m)')
title('GPS卫星MW组合')
grid on
legend('Location', 'best')

subplot(2,1,2)
hold on
for i = 1:length(gps_idx)
    plot(full_time/3600, GF_matrix(:,gps_idx(i)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', valid_prns{gps_idx(i)})
end
xlabel('时间 (小时)')
ylabel('GF (m)')
title('GPS卫星GF组合')
grid on

% 提取北斗卫星索引
bds_geo_idx = find(ismember(valid_prns, bds_geo_prns));
bds_igso_idx = find(ismember(valid_prns, bds_igso_prns));
bds_meo_idx = find(ismember(valid_prns, bds_meo_prns));
figure;
subplot(2,1,1)
hold on
% 绘制GEO
for i = 1:length(bds_geo_idx)
    plot(full_time/3600, MW_matrix(:,bds_geo_idx(i)), ...
        'Color', colors(i,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_geo_idx(i)} ' (GEO)'])
end
% 绘制IGSO
for i = 1:length(bds_igso_idx)
    plot(full_time/3600, MW_matrix(:,bds_igso_idx(i)), ...
        'Color', colors(i+2,:), 'LineStyle', line_styles{2}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_igso_idx(i)} ' (IGSO)'])
end
% 绘制MEO
for i = 1:length(bds_meo_idx)
    plot(full_time/3600, MW_matrix(:,bds_meo_idx(i)), ...
        'Color', colors(i+4,:), 'LineStyle', line_styles{3}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_meo_idx(i)} ' (MEO)'])
end
xlabel('时间 (小时)')
ylabel('MW (m)')
title('北斗卫星MW组合')
grid on
legend('Location', 'best')

subplot(2,1,2)
hold on
% 绘制GEO
for i = 1:length(bds_geo_idx)
    plot(full_time/3600, GF_matrix(:,bds_geo_idx(i)), ...
        'Color', colors(i,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_geo_idx(i)} ' (GEO)'])
end
% 绘制IGSO
for i = 1:length(bds_igso_idx)
    plot(full_time/3600, GF_matrix(:,bds_igso_idx(i)), ...
        'Color', colors(i+2,:), 'LineStyle', line_styles{2}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_igso_idx(i)} ' (IGSO)'])
end
% 绘制MEO
for i = 1:length(bds_meo_idx)
    plot(full_time/3600, GF_matrix(:,bds_meo_idx(i)), ...
        'Color', colors(i+4,:), 'LineStyle', line_styles{3}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_meo_idx(i)} ' (MEO)'])
end
xlabel('时间 (小时)')
ylabel('GF (m)')
title('北斗卫星GF组合')
grid on
MW_diff = diff(MW_matrix, 1, 1);
GF_diff = diff(GF_matrix, 1, 1);
full_time_diff = full_time(1:end-1) + 15; % 差分时间点（30秒间隔的中点）
figure;
subplot(2,1,1)
hold on
for i = 1:length(gps_idx)
    plot(full_time_diff/3600, MW_diff(:,gps_idx(i)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', valid_prns{gps_idx(i)})
end
xlabel('时间 (小时)'), ylabel('\Delta MW (m)')
title('GPS卫星MW组合差分'), grid on, legend('Location', 'best')

subplot(2,1,2)
hold on
for i = 1:length(gps_idx)
    plot(full_time_diff/3600, GF_diff(:,gps_idx(i)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', valid_prns{gps_idx(i)})
end
xlabel('时间 (小时)'), ylabel('\Delta GF (m)')
title('GPS卫星GF组合差分'), grid on

figure;
subplot(2,1,1)
hold on
% GEO
for i = 1:length(bds_geo_idx)
    plot(full_time_diff/3600, MW_diff(:,bds_geo_idx(i)), ...
        'Color', colors(i,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_geo_idx(i)} ' (GEO)'])
end
% IGSO
for i = 1:length(bds_igso_idx)
    plot(full_time_diff/3600, MW_diff(:,bds_igso_idx(i)), ...
        'Color', colors(i+2,:), 'LineStyle', line_styles{2}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_igso_idx(i)} ' (IGSO)'])
end

for i = 1:length(bds_meo_idx)
    plot(full_time_diff/3600, MW_diff(:,bds_meo_idx(i)), ...
        'Color', colors(i+4,:), 'LineStyle', line_styles{3}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_meo_idx(i)} ' (MEO)'])
end
xlabel('时间 (小时)'), ylabel('\Delta MW (m)')
title('北斗卫星MW组合差分'), grid on, legend('Location', 'best')

subplot(2,1,2)
hold on

for i = 1:length(bds_geo_idx)
    plot(full_time_diff/3600, GF_diff(:,bds_geo_idx(i)), ...
        'Color', colors(i,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_geo_idx(i)} ' (GEO)'])
end
for i = 1:length(bds_igso_idx)
    plot(full_time_diff/3600, GF_diff(:,bds_igso_idx(i)), ...
        'Color', colors(i+2,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_igso_idx(i)} ' (GEO)'])
end
for i = 1:length(bds_meo_idx)
    plot(full_time_diff/3600, GF_diff(:,bds_meo_idx(i)), ...
        'Color', colors(i+4,:), 'LineStyle', line_styles{1}, 'LineWidth', 1.5, ...
        'DisplayName', [valid_prns{bds_meo_idx(i)} ' (GEO)'])
end

xlabel('时间 (小时)'), ylabel('\Delta GF (m)')
title('北斗卫星GF组合差分'), grid on
