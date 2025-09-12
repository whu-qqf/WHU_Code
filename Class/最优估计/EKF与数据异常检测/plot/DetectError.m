function utc_time = gps2utc(gps_week, seconds_of_week)
    gps_start = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
    total_seconds = gps_week * 604800 + seconds_of_week - 18;
    utc_time = gps_start + seconds(total_seconds);
end
week=2086;
%%
%alldata=importdata("result_5_error.txt");
alldata=importdata("result_5.txt");
data=alldata.data;
for i=1:5010
    time(i)=gps2utc(week,data(i,1));
end
figure;
subplot(3,1,1);
plot(time,data(:,7), 'b-', 'LineWidth', 1);
xlabel("UTC");
ylabel("East(m)");
E_m=mean(data(:,7));
E_rms=rms(data(:,7));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("东方向误差");

subplot(3,1,2);
plot(time,data(:,8),'-', 'LineWidth', 1);
xlabel("UTC");
ylabel("North(m)");
N_m=mean(data(:,8));
N_rms=rms(data(:,8));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', N_m, N_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("北方向误差");

subplot(3,1,3);
plot(time,data(:,9),'g-', 'LineWidth', 1);
xlabel("UTC");
ylabel("Up(m)");
U_m=mean(data(:,9));
U_rms=rms(data(:,9));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', U_m, U_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("天方向误差");
%%
%alldata=importdata("result_8_error.txt");
alldata=importdata("result_8.txt");
data=alldata.data;
figure;
subplot(3,1,1);
plot(time,data(:,10), 'b-', 'LineWidth', 1);
xlabel("UTC");
ylabel("East(m)");
E_m=mean(data(:,10));
E_rms=rms(data(:,10));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("东方向误差");

subplot(3,1,2);
plot(time,data(:,11),'-', 'LineWidth', 1);
xlabel("UTC");
ylabel("North(m)");
N_m=mean(data(:,11));
N_rms=rms(data(:,11));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', N_m, N_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("北方向误差");

subplot(3,1,3);
plot(time,data(:,12),'g-', 'LineWidth', 1);
xlabel("UTC");
ylabel("Up(m)");
U_m=mean(data(:,12));
U_rms=rms(data(:,12));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', U_m, U_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("天方向误差");
%%
%alldata=importdata("result_10_error.txt");
alldata=importdata("result_10_error_repair.txt");
%alldata=importdata("result_10.txt");

data=alldata.data;
figure;
subplot(3,1,1);
plot(data(:,1),data(:,12), 'b-', 'LineWidth', 1);
xlabel("UTC");
ylabel("East(m)");
E_m=mean(data(:,12));
E_rms=rms(data(:,12));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', E_m, E_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("东方向误差");

subplot(3,1,2);
plot(time,data(:,13),'-', 'LineWidth', 1);
xlabel("UTC");
ylabel("North(m)");
N_m=mean(data(:,13));
N_rms=rms(data(:,13));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', N_m, N_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("北方向误差");

subplot(3,1,3);
plot(time,data(:,14),'g-', 'LineWidth', 1);
xlabel("UTC");
ylabel("Up(m)");
U_m=mean(data(:,14));
U_rms=rms(data(:,14));
yline(E_m, 'r--', 'LineWidth', 2);
xlims = get(gca, 'XLim');
text_x = xlims(1) + 0.01 * (xlims(2) - xlims(1));
text(text_x, E_m, sprintf('平均: %.2f\nRMS: %.2f', U_m, U_rms), 'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');
title("天方向误差");