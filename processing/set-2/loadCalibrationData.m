function [accelerometer, magnetometer] =  loadCalibrationData
% loadCalibrationData Loads sensor calibration data and populates global variables with the data to be used by the calibration functions.

    %% Load magnetometer data
    global compassCalibrationData
    if ~exist('compassCalibrationData', 'var') || isempty(compassCalibrationData)
        disp('Loading magnetometer calibration data.');
        compassCalibrationData = loadCompassCalibationData();
    else
        %disp('Using existing compass calibration data.');
    end
    magnetometer = compassCalibrationData;

    %% Load accelerometer data
    global accelerometerCalibrationData
    if ~exist('accelerometerCalibrationData', 'var') || isempty(accelerometerCalibrationData)
        disp('Loading accelerometer calibration data.');
        accelerometerCalibrationData = loadAccelerometerCalibrationData();
    else
        %disp('Using existing accelerometer calibration data.');
    end
    accelerometer = accelerometerCalibrationData;
    
end

%% Magnetometer calibration data
function data = loadCompassCalibationData
    
    % define the data set folder
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'full-sphere');

    % Load the data
    [~, ~, magnetometer, ~] = loadData(dataSetFolder);

    % Fetch axes
    x = magnetometer.Data(:, 1);
    y = magnetometer.Data(:, 2);
    z = magnetometer.Data(:, 3);
    
    % Calibrate sensor
    correction = calibrateByEllipseFitting(x, y, z);

    disp(' ');
    disp('Affine hard- and soft-iron correction matrix:');
    disp(num2str(correction));
    
    % Return data
    data = struct('correctionMatrix', correction);
    
end

%% Accelerometer calibration data
function data = loadAccelerometerCalibrationData

    % Load the data: x pointing forward
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
    [accelerometerZup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: x pointing up
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-z-pointing-forward-and-x-up');
    [accelerometerXup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: z pointing left
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-z-pointing-forward-and-y-down');
    [accelerometerZleft, ~, ~, ~] = loadData(dataSetFolder);

    % Determine x-axis offset and variance
    x = [accelerometerZup.Data(:,1); accelerometerZleft.Data(:,1)];
    xMean       = mean(x);
    xVariance   = var(x);
    
    % Determine y-axis offset and variance
    y = [accelerometerZup.Data(:,2); accelerometerXup.Data(:,2)];
    yMean       = mean(y);
    yVariance   = var(y);
    
    % Determine z-axis offset and variance
    z = [accelerometerXup.Data(:,3); accelerometerZleft.Data(:,3)];
    zMean       = mean(z);
    zVariance   = var(z);
    
    % Prepare packages
    means = [xMean, yMean, zMean];
    variances = [xVariance, yVariance, zVariance];
    
    disp('Calculated axis offset: ');
    disp(num2str(means));
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));
    
    %% Calibrate using ellipsoid fitting
    
     % define the data set folder
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'full-sphere');

    % Load the data
    [accelerometer, ~, ~, ~] = loadData(dataSetFolder);
    
    % Fetch axes
    x = accelerometer.Data(:, 1);
    y = accelerometer.Data(:, 2);
    z = accelerometer.Data(:, 3);
    
    % Calibrate sensor
    correction = calibrateByEllipseFitting(x, y, z);

    disp(' ');
    disp('Affine correction matrix from ellipsoid fitting:');
    disp(num2str(correction));
    
    %% Return values
    
    % Return data
    data = struct(...
        'offset', means, ...
        'variance', variances, ...
        'correctionMatrix', correction ...
        );
    
end