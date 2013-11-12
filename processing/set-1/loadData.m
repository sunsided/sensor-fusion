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

    accel = accelBuffer;
    gyro = gyroBuffer;
    compass = compassBuffer;
    temp = temperatureBuffer;
    
end