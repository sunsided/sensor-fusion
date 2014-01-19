function [accelerometer, gyroscope, magnetometer] =  loadCalibrationData
% loadCalibrationData Loads sensor calibration data and populates global variables with the data to be used by the calibration functions.

    path(fullfile(fileparts(which(mfilename)), '..'), path);

    % Load magnetometer data
    global compassCalibrationData
    if ~exist('compassCalibrationData', 'var') || isempty(compassCalibrationData)
        disp('Loading magnetometer calibration data.');
        compassCalibrationData = loadCompassCalibationData();
    end
    magnetometer = compassCalibrationData;

    % Load accelerometer data
    global accelerometerCalibrationData
    if ~exist('accelerometerCalibrationData', 'var') || isempty(accelerometerCalibrationData)
        disp('Loading accelerometer calibration data.');
        accelerometerCalibrationData = loadAccelerometerCalibrationData();
    end
    accelerometer = accelerometerCalibrationData;
    
     % Load gyroscope data
    global gyroscopeCalibrationData
    if ~exist('gyroscopeCalibrationData', 'var') || isempty(gyroscopeCalibrationData)
        disp('Loading gyroscope calibration data.');
        gyroscopeCalibrationData = loadGyroscopeCalibrationData();
    end
    gyroscope = gyroscopeCalibrationData;
    
end