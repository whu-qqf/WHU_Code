function utc_time = gps2utc(gps_week, seconds_of_week)
    gps_start = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
    total_seconds = gps_week * 604800 + seconds_of_week - 18;
    utc_time = gps_start + seconds(total_seconds);
end
%%
alldata=importdata("ENU.txt");
data=alldata.data;
for i=1:length(data)
    time(i)=gps2utc(data(i,1),data(i,2))+hours(8);
end
figure;
subplot(3,1,1);
plot(time,data(:,3), 'b-', 'LineWidth', 1);
xlabel("UTC+8h");
ylabel("East(m)");
ylim([-2 2]);
E_m=mean(data(:,3));
E_rms=rms(data(:,3));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("东方向误差");
grid on;
subplot(3,1,2);
plot(time,data(:,4),'-', 'LineWidth', 1);
xlabel("UTC+8h");
ylabel("North(m)");
ylim([-2 2])
N_m=mean(data(:,4));
N_rms=rms(data(:,4));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', N_m, N_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("北方向误差");
grid on;
subplot(3,1,3);
plot(time,data(:,5),'g-', 'LineWidth', 1);
xlabel("UTC+8h");
ylabel("Up(m)");
ylim([-4 4])
U_m=mean(data(:,5));
U_rms=rms(data(:,5));
yline(U_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, U_m, sprintf('平均: %.2f\nRMS: %.2f', U_m, U_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("天方向误差");
grid on;
%%
figure;
plot(time,data(:,6),'-o');
