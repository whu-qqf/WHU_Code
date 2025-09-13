% 生成10个测站的模拟误差数据（单位：厘米）
stationNames = {'D010', 'D009', 'E169', 'E103', 'E007', 'E012', 'E001', 'E002', 'D045', 'D048'};
errorData = [0,-0.8,-4.5;
    0.7,0.3,-6.2;
    1.2,-3.1,-0.3;
    1.5,-1,-4;
    1.4,-2.7,-2.7;
    -0.1,-2.6,-4.2;
    3.6,-5.2,1;
    0.5,-2.6,-3.8;
    1.18,-1.1,-3.17;
    1.59,1.4,-1.6];
figure('Color', [1 1 1], 'Position', [50, 50, 1200, 700]);
ax = gca;
set(ax, 'FontName', 'Arial', 'FontSize', 10, 'FontWeight', 'normal', ...
    'LineWidth', 1.0, 'Box', 'on', 'GridAlpha', 0.1, ...
    'XColor', [0.3 0.3 0.3], 'YColor', [0.3 0.3 0.3], ...
    'YGrid', 'on', 'XGrid', 'off', 'GridLineStyle', '-', 'GridColor', [0.9 0.9 0.9]);
hBar = bar(errorData, 'grouped', 'BarWidth', 0.85);
colors = [
    0.00 0.35 0.70;  % 蓝色 - 东方向
    0.10 0.60 0.25;  % 绿色 - 北方向
    0.85 0.25 0.10   % 橙色 - 天方向
];
% 设置柱状图样式
for k = 1:3
    hBar(k).FaceColor = 'flat';
    hBar(k).CData = repmat(colors(k, :), size(errorData, 1), 1);
    hBar(k).EdgeColor = 'none';
    hBar(k).FaceAlpha = 0.9;
end
% 添加零基准线
yline(0, 'LineWidth', 1.2, 'Color', [0.4 0.4 0.4], 'Alpha', 0.6);
for i = 1:size(errorData, 1)
    for j = 1:size(errorData, 2)
        value = errorData(i,j);
        offset = sign(value) * 0.1; % 根据正负调整标签位置
        if abs(value) < 0.5  % 小数值特殊处理
            vertAlign = 'middle';
            yPos = value;
            textColor = [0.3 0.3 0.3]; % 灰色
            bgColor = [1 1 1 0.8]; % 半透明白背景
            edgeColor = [0.7 0.7 0.7];
        elseif value >= 0
            vertAlign = 'bottom';
            yPos = value + 0.15;
            textColor = [0.0 0.4 0.0]; % 深绿色
            bgColor = 'none';
            edgeColor = 'none';
        else
            vertAlign = 'top';
            yPos = value - 0.15;
            textColor = [0.7 0.0 0.0]; % 深红色
            bgColor = 'none';
            edgeColor = 'none';
        end
        text(i + (j-2.1)*0.22, yPos, sprintf('%+.1f', value), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', vertAlign, ...
            'FontSize', 8, 'FontWeight', 'bold', ...
            'Color', textColor, ...
            'BackgroundColor', bgColor, ...
            'EdgeColor', edgeColor, ...
            'Margin', 0.5);
    end
end

% 设置坐标轴和标签
xlabel('测站名称', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);
ylabel('误差 (cm)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);
title('10个测站ENU方向误差分析', 'FontSize', 14, 'FontWeight', 'bold', ...
    'Color', [0.15 0.15 0.45]);
xticks(1:length(stationNames));
xticklabels(stationNames);
ax.XAxis.FontSize = 9; % 减小X轴标签字体防止重叠
ax.XAxis.TickLabelRotation = 15; % 旋转标签防止重叠
maxError = max(abs(errorData(:)));
ylim([-maxError-1, maxError+1]);
yticks(floor(-maxError-1):0.5:ceil(maxError+1));
lgd = legend({'东方向', '北方向', '天方向'}, ...
    'Location', 'northeastoutside', 'FontSize', 10, 'Box', 'off');
lgd.Color = [1 1 1 0.7]; % 半透明背景

annotation('textbox', [0.05, 0.92, 0.3, 0.04], ...
    'String', 'ENU: 东(East)、北(North)、天(Up)方向', ...
    'EdgeColor', 'none', ...
    'FontSize', 8, ...
    'Color', [0.4 0.4 0.4]);

% 添加数据点标记
hold on;
markerSize = 40;
eastMean=mean(errorData(:,1));
northMean=mean(errorData(:,2));
upMean=mean(errorData(:,3));
% 添加均值线
plot(1:10, repmat(eastMean,1,10), '--', 'Color', [0 0.3 0.7 0.5], 'LineWidth', 1.2);
plot(1:10, repmat(northMean,1,10), '--', 'Color', [0 0.5 0.2 0.5], 'LineWidth', 1.2);
plot(1:10, repmat(upMean,1,10), '--', 'Color', [0.8 0.2 0 0.5], 'LineWidth', 1.2);
legend("东方向误差","北方向误差","天方向误差","东方向均值","北方向均值","天方向均值");
% 调整布局
set(gca, 'Position', [0.08 0.15 0.7 0.75]); 
grid on;
disp('10个测站ENU误差分析图绘制完成！');