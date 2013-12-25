function [accel, gyro, compass, temp] = loadData(dataSetFolder, calibrate)

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

    % create local copies as calls to calibrare... will overwrite them
    accelerometer = accelBuffer;
    gyroscope = gyroBuffer;
    magnetometer = compassBuffer;
    temperature = temperatureBuffer;
    
    % calibrate if requested
    if exist('calibrate', 'var') && calibrate == true
        path(fullfile(fileparts(which(mfilename)), 'calibration'), path);
        
        [accelerometer(:,2:4), accCalibration] = calibrateAccelerometer(accelerometer(:,2:4));
        [gyroscope(:,2:4), gyroCalibration] = calibrateGyroscope(gyroscope(:,2:4));        
        [magnetometer(:,2:4), magCalibration] = calibrateMagnetometer(magnetometer(:,2:4));
    else
        accCalibration = [];
        gyroCalibration = [];
        magCalibration = [];
    end
    
    % construct timeseries objects
    accel = timeseries(accelerometer(:,2:4), accelerometer(:,1), 'Name', 'Accelerometer');
    gyro = timeseries(gyroscope(:,2:4), gyroscope(:,1), 'Name', 'Gyroscope');
    compass = timeseries(magnetometer(:,2:4), magnetometer(:,1), 'Name', 'Magnetometer');
    temp = timeseries(temperature(:,2), temperature(:,1), 'Name', 'Temperature');
    
    accel.UserData = accCalibration;
    gyro.UserData = gyroCalibration;
    compass.UserData = magCalibration;
    
end