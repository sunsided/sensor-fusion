clear
clc

% Fetch all data.
load('hmc5883l_compass.mat');
load('mpu6050_accelerometer.mat');
load('mpu6050_gyroscope.mat');
load('mpu6050_temperature.mat');

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
    temperatureStats, temperatureLinearity, temperatureTimeStats;

    accelXStats, accelXLinearity, accelTimeStats;
    accelYStats, accelYLinearity, accelTimeStats;
    accelZStats, accelZLinearity, accelTimeStats;
    
    gyroXStats, gyroXLinearity, gyroTimeStats;
    gyroYStats, gyroYLinearity, gyroTimeStats;
    gyroZStats, gyroZLinearity, gyroTimeStats;
        
    magnetometerXStats, magnetometerXLinearity, magnetometerTimeStats;
    magnetometerYStats, magnetometerYLinearity, magnetometerTimeStats;
    magnetometerZStats, magnetometerZLinearity, magnetometerTimeStats;
];

T = array2table(dataCombined, 'VariableNames', {
    % 'sensor', 'type', 'axis', ...
    'v_min', 'v_max', 'v_range', 'v_diff_mean', 'v_diff_std', 'v_mean', 'v_std_dev', ...
    'linear_slope', 'linear_intercept', 'linear_RMSE', 'linear_R_squared', ...
    't_min', 't_max', 't_range', 'freq_mean', 'sample_rate_mean', 'sample_rate_std'
    });

% Write CSV manually to control formatting
fid = fopen('stats.csv', 'w');
fprintf(fid, 'sensor,type,axis,v_min,v_max,v_range,v_diff_mean,v_diff_std,v_mean,v_std_dev,linear_slope,linear_intercept,linear_RMSE,linear_R_squared,t_min,t_max,t_range,freq_mean,sample_rate_mean,sample_rate_std\n');
for row = 1:height(T)
    fprintf(fid, '%s,%s,%s,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.2f,%.6g,%.6g\n', ...
        sensors(:, row), sensorTypes(row), axes(row), ...
        T.v_min(row), T.v_max(row), T.v_range(row), T.v_diff_mean(row), T.v_diff_std(row), T.v_mean(row), T.v_std_dev(row), ...
        T.linear_slope(row), T.linear_intercept(row), T.linear_RMSE(row), T.linear_R_squared(row), ...
        T.t_min(row), T.t_max(row), T.t_range(row), T.freq_mean(row), T.sample_rate_mean(row), T.sample_rate_std(row));
end
fclose(fid);

% Read it back for visualization.
readtable('stats.csv')

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
