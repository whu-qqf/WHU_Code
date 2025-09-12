% 打开文件
fileID = fopen('Satellite.txt', 'r'); 
formatSpec = '%s %f %f %f %f'; 
data = textscan(fileID, formatSpec, 'MultipleDelimsAsOne', 1);
fclose(fileID);
BroadSatData = table(data{1}, data{2}, data{3}, data{4}, data{5}, ...
    'VariableNames', {'Satellite', 'Daysecond', 'X', 'Y', 'Z'});

uniqueSats = unique(BroadSatData.Satellite);
fileID = fopen('PreSatellite.txt', 'r'); 
data = textscan(fileID, formatSpec, 'MultipleDelimsAsOne', 1);
fclose(fileID);
PreSatData = table(data{1}, data{2}, data{3}, data{4}, data{5}, ...
    'VariableNames', {'Satellite', 'Daysecond', 'X', 'Y', 'Z'});

clear data;
clear ans;
clear fileID;
clear formatSpec;


BroadSatGroups = struct();
%
for i = 1:length(uniqueSats)
    satName = uniqueSats{i};
    mask = strcmp(BroadSatData.Satellite, satName);  % 逻辑索引
    BroadSatGroups.(satName) = BroadSatData(mask, :);     % 存储
end

uniqueSats = unique(PreSatData.Satellite);
PreSatGroups = struct();
for i = 1:length(uniqueSats)
    satName = uniqueSats{i};
    mask = strcmp(PreSatData.Satellite, satName);  % 逻辑索引
    PreSatGroups.(satName) = PreSatData(mask, :);     % 存储分组数据
end
clear i;
clear mask;
clear satList;
clear satName;
clear uniqueSats;
clear groupID;
satNames=fieldnames(BroadSatGroups);
t_interp=(0:86400)';
figure;

for i=1:length(satNames)
    satName=satNames{i};
    origTable=BroadSatGroups.(satName);
    [t_orig, sortIdx] = sort(origTable.Daysecond);
    X_orig = origTable.X(sortIdx);
    Y_orig = origTable.Y(sortIdx);
    Z_orig = origTable.Z(sortIdx);
    scatter3(origTable.X,origTable.Y,origTable.Z,10,'filled');
    BroadSatGroups.(satName) = struct(...
        'OriginalData', origTable, ...       
        'InterpData', table(t_interp, X_orig, Y_orig, Z_orig, ...
            'VariableNames', {'Time', 'X', 'Y', 'Z'}) ...
    );
    title("广播星历轨迹图");
    hold on;
end
hold off;
satNames = fieldnames(PreSatGroups);
t_interp = (0:86400)';
figure;
for i = 1:length(satNames)
    satName = satNames{i};
    origTable = PreSatGroups.(satName); 
    [t_orig, sortIdx] = sort(origTable.Daysecond);
    X_orig = origTable.X(sortIdx);
    Y_orig = origTable.Y(sortIdx);
    Z_orig = origTable.Z(sortIdx);
    X_interp = interp1(t_orig, X_orig, t_interp, 'spline');
    Y_interp = interp1(t_orig, Y_orig, t_interp, 'spline');
    Z_interp = interp1(t_orig, Z_orig, t_interp, 'spline');
    scatter3(X_interp,Y_interp,Z_interp,10,'filled');
    title("精密星历轨迹图");
    hold on;
  
    PreSatGroups.(satName) = struct(...
        'OriginalData', origTable, ...       
        'InterpData', table(t_interp, X_interp, Y_interp, Z_interp, ...
            'VariableNames', {'Time', 'X', 'Y', 'Z'}) ...
    );

end
hold off;

%%%%%%%
%%
% 每秒间隔
x_error=(1800:83700);
y_error=(1800:83700);
z_error=(1800:83700);
date=datetime(2019,12,1,0,0,0);
dates=date+seconds(1800:1:83700);


figure;
subplot(3,1,1);
ending=length(satNames);
for j=32:36

    for i=1800:83700
        x_error(i-1799)=PreSatGroups.(satNames{j}).InterpData.X(i)-BroadSatGroups.(satNames{j}).InterpData.X(i);
    end
    plot(dates,x_error);
    xlabel("UTC时间");
    ylabel("X方向误差(m)");
    hold on;
end
hold off;
subplot(3,1,2);
for j=32:36
    for i=1800:83700
        y_error(i-1799)=PreSatGroups.(satNames{j}).InterpData.Y(i)-BroadSatGroups.(satNames{j}).InterpData.Y(i);
    end
    plot(dates,y_error);
    xlabel("UTC时间");
    ylabel("Y方向误差(m)");
    hold on;
end
hold off;
subplot(3,1,3);
for j=32:36

    for i=1800:83700
        z_error(i-1799)=PreSatGroups.(satNames{j}).InterpData.Z(i)-BroadSatGroups.(satNames{j}).InterpData.Z(i);
    end
    plot(dates,z_error);
    xlabel("UTC时间");
    ylabel("Z方向误差(m)");
    hold on;
end
hold off;
%%
% 15min间隔
x_error=(1:96);
y_error=(1:96);
z_error=(1:96);
date=datetime(2019,12,1,0,0,0);
dates=date+seconds(0:900:85500);
figure;
subplot(3,1,1);
ending=length(satNames);
for j=1:ending

    for i=1:96
        x_error(i)=PreSatGroups.(satNames{j}).InterpData.X((i-1)*900+1)-BroadSatGroups.(satNames{j}).InterpData.X((i-1)*900+1);
    end
    plot(dates,x_error);
    xlabel("UTC时间");
    ylabel("X方向误差(m)");
    hold on;
end
hold off;

subplot(3,1,2);
for j=1:ending

    for i=1:96
        y_error(i)=PreSatGroups.(satNames{j}).InterpData.Y((i-1)*900+1)-BroadSatGroups.(satNames{j}).InterpData.Y((i-1)*900+1);
    end
    plot(dates,y_error);
    xlabel("UTC时间");
    ylabel("Y方向误差(m)");
    hold on;
end
hold off;
subplot(3,1,3);
for j=1:ending

    for i=1:96
        z_error(i)=PreSatGroups.(satNames{j}).InterpData.Z((i-1)*900+1)-BroadSatGroups.(satNames{j}).InterpData.Z((i-1)*900+1);
    end
    plot(dates,z_error);
    xlabel("UTC时间");
    ylabel("Z方向误差(m)");
    hold on;
end
hold off;
