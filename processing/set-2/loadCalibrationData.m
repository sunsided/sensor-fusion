function [accelerometer, magnetometer] =  loadCalibrationData
% loadCalibrationData Loads sensor calibration data and populates global variables with the data to be used by the calibration functions.

    % Load calibration functions
    path(fullfile(fileparts(which(mfilename)), 'calibration'), path);

    % Load magnetometer data
    global compassCalibrationData
    if ~exist('compassCalibrationData', 'var') || isempty(compassCalibrationData)
        disp('Loading magnetometer calibration data.');
        compassCalibrationData = loadCompassCalibationData();
    else
        %disp('Using existing compass calibration data.');
    end
    magnetometer = compassCalibrationData;

    % Load accelerometer data
    global accelerometerCalibrationData
    if ~exist('accelerometerCalibrationData', 'var') || isempty(accelerometerCalibrationData)
        disp('Loading accelerometer calibration data.');
        accelerometerCalibrationData = loadAccelerometerCalibrationData();
    else
        %disp('Using existing accelerometer calibration data.');
    end
    accelerometer = accelerometerCalibrationData;
    
end