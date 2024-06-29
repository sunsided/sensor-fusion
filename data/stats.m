%% Load data

clear
clc

% Fetch all data.
load('hmc5883l_compass.mat');
load('mpu6050_accelerometer.mat');
load('mpu6050_gyroscope.mat');
load('mpu6050_temperature.mat');

%% Export statistics as CSV

% Sampling rates.
[t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std] = timeStats(accelBuffer);
accelTimeStats = [t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std];
[t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std] = timeStats(gyroBuffer);
gyroTimeStats = [t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std];
[t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std] = timeStats(temperatureBuffer);
temperatureTimeStats = [t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std];
[t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std] = timeStats(compassBuffer);
magnetometerTimeStats = [t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std];

clear t_min t_max t_duration sampling_frequency sampling_rate sampling_rate_std

% Value statistics: Accelerometer
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(accelBuffer, 2);
accelXStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(accelBuffer, 3);
accelYStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(accelBuffer, 4);
accelZStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];

% Value statistics: Gyroscope
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(gyroBuffer, 2);
gyroXStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(gyroBuffer, 3);
gyroYStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(gyroBuffer, 4);
gyroZStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];

% Value statistics: Magnetometer
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(compassBuffer, 2);
magnetometerXStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(compassBuffer, 3);
magnetometerYStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(compassBuffer, 4);
magnetometerZStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];

% Value statistics: Temperature
[v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(temperatureBuffer, 2);
temperatureStats = [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev];

clear v_min v_max v_range v_diff_mean v_diff_std v_mean v_std_dev

% Measure linearity: Acelerometer
[slope, intercept, RMSE, R_squared] = linearFit(accelBuffer, 2);
accelXLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(accelBuffer, 3);
accelYLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(accelBuffer, 4);
accelZLinearity = [slope, intercept, RMSE, R_squared];

% Measure linearity: Gyroscope
[slope, intercept, RMSE, R_squared] = linearFit(gyroBuffer, 2);
gyroXLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(gyroBuffer, 3);
gyroYLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(gyroBuffer, 4);
gyroZLinearity = [slope, intercept, RMSE, R_squared];

% Measure linearity: Magnetometer
[slope, intercept, RMSE, R_squared] = linearFit(compassBuffer, 2);
magnetometerXLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(compassBuffer, 3);
magnetometerYLinearity = [slope, intercept, RMSE, R_squared];
[slope, intercept, RMSE, R_squared] = linearFit(compassBuffer, 4);
magnetometerZLinearity = [slope, intercept, RMSE, R_squared];

% Measure linearity: Temperature
[slope, intercept, RMSE, R_squared] = linearFit(temperatureBuffer, 2);
temperatureLinearity = [slope, intercept, RMSE, R_squared];

clear slope intercept RMSE R_squared

% Wrap the data into a table and export as CSV
sensors = ["MPU6050", ...
           'MPU6050', 'MPU6050', 'MPU6050', ...
           'MPU6050', 'MPU6050', 'MPU6050', ...
           'HMC5833L', 'HMC5833L', 'HMC5833L'];
sensorTypes = ["temperature", ...
            'accelerometer', 'accelerometer', 'accelerometer', ...
            'gyroscope', 'gyroscope', 'gyroscope', ...
            'magnetometer', 'magnetometer', 'magnetometer'];
axes = ['T', ...
    'X', 'Y', 'Z', ...
    'X', 'Y', 'Z', ...
    'X', 'Y', 'Z'];

dataCombined = [
    size(temperatureBuffer, 1), temperatureStats, temperatureLinearity, temperatureTimeStats;

    size(accelBuffer, 1), accelXStats, accelXLinearity, accelTimeStats;
    size(accelBuffer, 1), accelYStats, accelYLinearity, accelTimeStats;
    size(accelBuffer, 1), accelZStats, accelZLinearity, accelTimeStats;
    
    size(gyroBuffer, 1), gyroXStats, gyroXLinearity, gyroTimeStats;
    size(gyroBuffer, 1), gyroYStats, gyroYLinearity, gyroTimeStats;
    size(gyroBuffer, 1), gyroZStats, gyroZLinearity, gyroTimeStats;
        
    size(compassBuffer, 1), magnetometerXStats, magnetometerXLinearity, magnetometerTimeStats;
    size(compassBuffer, 1), magnetometerYStats, magnetometerYLinearity, magnetometerTimeStats;
    size(compassBuffer, 1), magnetometerZStats, magnetometerZLinearity, magnetometerTimeStats;
];

T = array2table(dataCombined, 'VariableNames', {
    % 'sensor', 'type', 'axis', ...
    'count', 'v_min', 'v_max', 'v_range', 'v_diff_mean', 'v_diff_std', 'v_mean', 'v_std_dev', ...
    'linear_slope', 'linear_intercept', 'linear_RMSE', 'linear_R_squared', ...
    't_min', 't_max', 't_range', 'freq_mean', 'sample_rate_mean', 'sample_rate_std'
    });

% Write CSV manually to control formatting
fid = fopen('sensor-stats.csv', 'w');
fprintf(fid, 'sensor,type,axis,count,v_min,v_max,v_range,v_diff_mean,v_diff_std,v_mean,v_std_dev,linear_slope,linear_intercept,linear_RMSE,linear_R_squared,t_min,t_max,t_range,freq_mean,sample_rate_mean,sample_rate_std\n');
for row = 1:height(T)
    fprintf(fid, '%s,%s,%s,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.2f,%.6g,%.6g\n', ...
        sensors(:, row), sensorTypes(row), axes(row), ...
        T.count(row), T.v_min(row), T.v_max(row), T.v_range(row), T.v_diff_mean(row), T.v_diff_std(row), T.v_mean(row), T.v_std_dev(row), ...
        T.linear_slope(row), T.linear_intercept(row), T.linear_RMSE(row), T.linear_R_squared(row), ...
        T.t_min(row), T.t_max(row), T.t_range(row), T.freq_mean(row), T.sample_rate_mean(row), T.sample_rate_std(row));
end
fclose(fid);

% Read it back for visualization.
readtable('sensor-stats.csv')

%% Covariances

% Interpolate and extrapolate HMC5833L data to match MPU6050 sampling rate.
magnetometerInterp = interp1(compassBuffer(:, 1), compassBuffer(:, 2:end), accelBuffer(:, 1), 'previous');
magnetometerInterp = fillmissing(magnetometerInterp, 'next');
magnetometerInterp = fillmissing(magnetometerInterp, 'previous');

% Calculate the covariance.
sensorMatrix = [ temperatureBuffer(:, 2) accelBuffer(:, 2:end), gyroBuffer(:, 2:end), magnetometerInterp ];
sensorMatrixNormalized = zscore(sensorMatrix);
covarianceMatrix = cov(sensorMatrixNormalized);

% Write CSV manually to control formatting
fid = fopen('sensor-covariances.csv', 'w');
fprintf(fid, 'pair,temperature,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,magnetometer_x,magnetometer_y,magnetometer_z\n');
row_names = ["temperature", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "magnetometer_x", "magnetometer_y", "magnetometer_z"];
size = size(covarianceMatrix);
for row = 1:size(1)
    row_name = row_names(row);
    fprintf(fid, '%s,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', ...
        row_name, ...
        covarianceMatrix(row, 1), ...
        covarianceMatrix(row, 2), covarianceMatrix(row, 3), covarianceMatrix(row, 4), ...
        covarianceMatrix(row, 5), covarianceMatrix(row, 6), covarianceMatrix(row, 7), ...
        covarianceMatrix(row, 8), covarianceMatrix(row, 9), covarianceMatrix(row, 10));
end
fclose(fid);

close all;

figure;
heatmap(covarianceMatrix, 'ColorLimits', [-1 1], 'Colormap', parula, ...
    'XDisplayLabels', {'Temperature', 'Accelerometer X', 'Accelerometer Y', 'Accelerometer Z', 'Gyroscope X', 'Gyroscope Y', 'Gyroscope Y', 'Magnetometer X', 'Magnetometer Y', 'Magnetometer Z'}, ...
    'YDisplayLabels', {'Temperature', 'Accelerometer X', 'Accelerometer Y', 'Accelerometer Z', 'Gyroscope X', 'Gyroscope Y', 'Gyroscope Y', 'Magnetometer X', 'Magnetometer Y', 'Magnetometer Z'});
title('Sensor Covariance Heatmap');
set(gcf, 'Position', [100, 100, 800, 600]);  % [left, bottom, width, height]
saveas(gcf, 'sensor-covariances.png');

%% Plot axes over time

close all;

figure;
% Plot accelerometer data (row 1)
for i = 1:3  % Loop through x, y, z axes
    subplot(4, 3, i);  % Subplot for accelerometer
    plot(accelBuffer(:, 1), accelBuffer(:, i + 1));
    title(['Accelerometer Axis ' char('X' + i - 1)]);
    xlabel('Time (s)');
    ylabel('Acceleration (g)');  % 1g = 9.81 m/s^2
    grid on;
    
    axisMin = min(min(accelBuffer(:, 2:end)));
    axisMax = max(max(accelBuffer(:, 3:end)));
    axisLimit = max(abs([axisMin, axisMax])) * 1.5;
    ylim([-axisLimit, axisLimit]);
end

% Create figure for gyroscope plots (row 2)
for i = 1:3  % Loop through x, y, z axes
    subplot(4, 3, 3 + i);  % Subplot for gyroscope
    plot(gyroBuffer(:, 1), gyroBuffer(:, i + 1));
    title(['Gyroscope Axis ' char('X' + i - 1)]);
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    grid on;
    
    axisMin = min(min(gyroBuffer(:, 2:end)));
    axisMax = max(max(gyroBuffer(:, 3:end)));
    axisLimit = max(abs([axisMin, axisMax])) * 1.5;
    ylim([-axisLimit, axisLimit]);
end

% Create figure for magnetometer plots (row 3)
for i = 1:3  % Loop through x, y, z axes
    subplot(4, 3, 6 + i);  % Subplot for gyroscope
    plot(compassBuffer(:, 1), compassBuffer(:, i + 1));
    title(['Magnetometer Axis ' char('X' + i - 1)]);
    xlabel('Time (s)');
    ylabel('Mag. Field Strength (mG)');  % milli-Gauss
    grid on;
    
    axisMin = min(min(compassBuffer(:, 2:end)));
    axisMax = max(max(compassBuffer(:, 3:end)));
    axisLimit = max(abs([axisMin, axisMax])) * 1.5;
    ylim([-axisLimit, axisLimit]);
end

subplot(4, 3, 10:12);  % Subplot for temperature
plot(temperatureBuffer(:, 1), temperatureBuffer(:, 2));
title('Temperature');
xlabel('Time (s)');
ylabel('Temeprature (°C)');
grid on;

% Adjust figure properties
sgtitle('Sensor Readings');

set(gcf, 'Position', [100, 100, 1920, 1080]);
saveas(gcf, 'sensor-readings-fine.png');

set(gcf, 'Position', [100, 100, 1080, 1080]);
saveas(gcf, 'sensor-readings.png');

close all;

%% Helper functions

function [t_min, t_max, t_duration, sampling_frequency, sampling_rate, sampling_rate_std] = timeStats(buffer)
    t_min = buffer(1, 1);
    t_max = buffer(end, 1);
    t_duration = t_max - t_min;
    deltas = diff(buffer(:, 1));
    sampling_rate = mean(deltas);
    sampling_rate_std = std(deltas);
    sampling_frequency = 1  / sampling_rate;
end

function [v_min, v_max, v_range, v_diff_mean, v_diff_std, v_mean, v_std_dev] = valueStats(buffer, index)
    v_min = min(buffer(:, index));
    v_max = max(buffer(:, index));
    v_range = v_max - v_min;
    deltas = diff(buffer(:, index));
    v_diff_mean = mean(deltas);
    v_diff_std = std(deltas);
    v_mean = mean(buffer(:, index));
    v_std_dev = std(buffer(:, index));
end

function [slope, intercept, RMSE, R_squared] = linearFit(buffer, index)
    % Fit a line (i.e. first-order polynomial)
    x = buffer(:, 1);
    y = buffer(:, index);
    p = polyfit(x, y, 1);
    slope = p(1);
    intercept = p(2);
    
    % Evaluate polynomial.
    y_pred = polyval(p, x);
        
    % Calculate R-squared value (effect of time on the variance)
    SS_tot = sum((y - mean(y)).^2);     % Total sum of squares
    SS_res = sum((y - y_pred).^2);      % Residual sum of squares
    R_squared = 1 - (SS_res / SS_tot);  % R-squared value
    
    % RMSE
    RMSE = sqrt(mean((y - y_pred).^2));
end
