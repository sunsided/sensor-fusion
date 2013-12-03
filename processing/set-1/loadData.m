function [accel, gyro, compass, temp] = loadData(dataSetFolder)

    currentFolder = cd(dataSetFolder);
    try
        load mpu6050_accelerometer.mat
        load mpu6050_gyroscope.mat
        load mpu6050_temperature.mat
        load hmc5883l_compass.mat
    catch err
        dataSetFolder = pwd();
        cd(currentFolder);
        error('plotData:load', 'Error loading data from folder %s: %s', dataSetFolder, err.message);
    end
    cd(currentFolder);

    accel = timeseries(accelBuffer(:,2:4), accelBuffer(:,1), 'Name', 'Accelerometer');
    gyro = timeseries(gyroBuffer(:,2:4), gyroBuffer(:,1), 'Name', 'Gyroscope');
    compass = timeseries(compassBuffer(:,2:4), compassBuffer(:,1), 'Name', 'Magnetometer');
    temp = timeseries(temperatureBuffer(:,2), temperatureBuffer(:,1), 'Name', 'Temperature');
    
end