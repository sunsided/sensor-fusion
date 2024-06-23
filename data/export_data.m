clear

load('hmc5883l_compass.mat')
load('mpu6050_accelerometer.mat')
load('mpu6050_gyroscope.mat')
load('mpu6050_temperature.mat')

startTime = datetime(1970, 1, 1, 0, 0, 0);

% Export MPU6050 data as Parquet
mpu6050TimeIndex = startTime + seconds(accelBuffer(:, 1));
mpu6050Combined = [accelBuffer(:, 1), accelBuffer(:, 2:end), gyroBuffer(:, 2:end), temperatureBuffer(:, 2:end)];
T = array2table(mpu6050Combined, 'VariableNames', {'sec', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'temp'});
mpu6050TimeTable = table2timetable(T, 'RowTimes', mpu6050TimeIndex);
parquetwrite('mpu6050.parquet', mpu6050TimeTable);
writetable(T, 'mpu6050.csv')

% Export HMC5833L data as Parquet
hmc5883lTimeIndex = startTime + seconds(compassBuffer(:, 1));
hmc5883lCombined = [compassBuffer(:, 1), compassBuffer(:, 2:end)];
T = array2table(hmc5883lCombined, 'VariableNames', {'sec', 'compass_x', 'compass_y', 'compass_z'});
hmc5883lTimeTable = table2timetable(T, 'RowTimes', hmc5883lTimeIndex);
parquetwrite('hmc5883l.parquet', hmc5883lTimeTable);
writetable(T, 'hmc5833l.csv')