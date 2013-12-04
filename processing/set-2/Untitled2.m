
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-y-down');
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
    
    accelBuffer = accelBuffer(1990:end, :);
    gyroBuffer = gyroBuffer(1990:end, :);
    compassBuffer = compassBuffer(1560:end, :);
    temperatureBuffer = temperatureBuffer(1990:end, :);

    tstart = min(accelBuffer(1,1), compassBuffer(1,1));
    accelBuffer(:,1) = accelBuffer(:,1) - tstart;
    gyroBuffer(:,1) = gyroBuffer(:,1) - tstart;
    compassBuffer(:,1) = compassBuffer(:,1) - tstart;
    temperatureBuffer(:,1) = temperatureBuffer(:,1) - tstart;
    
save mpu6050_accelerometer.mat accelBuffer
save mpu6050_gyroscope.mat gyroBuffer
save mpu6050_temperature.mat temperatureBuffer
save hmc5883l_compass.mat compassBuffer
    
    accel = timeseries(accelBuffer(:,2:4), accelBuffer(:,1), 'Name', 'Accelerometer');
    gyro = timeseries(gyroBuffer(:,2:4), gyroBuffer(:,1), 'Name', 'Gyroscope');
    compass = timeseries(compassBuffer(:,2:4), compassBuffer(:,1), 'Name', 'Magnetometer');
    temp = timeseries(temperatureBuffer(:,2), temperatureBuffer(:,1), 'Name', 'Temperature');