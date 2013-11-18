function [accelerometer, magnetometer] =  loadCalibrationData
% loadCalibrationData Loads sensor calibration data and populates global variables with the data to be used by the calibration functions.

    %% Load magnetometer data
    global compassCalibrationData
    if ~exist('compassCalibrationData', 'var') || isempty(compassCalibrationData)
        disp('Loading magnetometer calibration data.');
        compassCalibrationData = loadCompassCalibationData();
    else
        disp('Using existing compass calibration data.');
    end
    magnetometer = compassCalibrationData;

    %% Load accelerometer data
    global accelerometerCalibrationData
    if ~exist('accelerometerCalibrationData', 'var') || isempty(accelerometerCalibrationData)
        disp('Loading accelerometer calibration data.');
        accelerometerCalibrationData = loadAccelerometerCalibrationData();
    else
        disp('Using existing accelerometer calibration data.');
    end
    accelerometer = accelerometerCalibrationData;
    
end

%% Magnetometer calibration data
function data = loadCompassCalibationData
    
    % define the data set folder
    dataSetFolder = '../../data/set-1/tilt-sphere';

    % Load the data
    [~, ~, magnetometer, ~] = loadData(dataSetFolder);

    % Fetch axes
    x = magnetometer(:, 2);
    y = magnetometer(:, 3);
    z = magnetometer(:, 4);    
    
    % Calculate hard-iron factor
    % In theory, a simple average (mean(magnetometer(:, 2:4)))
    % would do the trick, but that does not take into account the
    % actual number of measurements taken. Averging the component-wise
    % min and max values yields better results but is prone to errors
    % due to outliers in the measurement.
    hardIronOffset(1) = (max(x)+min(x))/2;
    hardIronOffset(2) = (max(y)+min(y))/2;
    hardIronOffset(3) = (max(z)+min(z))/2;
    
    disp('Calculated hard iron offset: ');
    disp(num2str(hardIronOffset));
    
    % Return data
    data = struct('hardIronOffset', hardIronOffset);
    
end

%% Accelerometer calibration data
function data = loadAccelerometerCalibrationData

    % Load the data: x pointing forward
    dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
    [accelerometerZup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: x pointing up
    dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
    [accelerometerXup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: z pointing left
    dataSetFolder = '../../data/set-1/unmoved-z-pointing-left';
    [accelerometerZleft, ~, ~, ~] = loadData(dataSetFolder);

    % Determine x-axis offset and variance
    x = [accelerometerZup(:,2); accelerometerZleft(:,2)];
    xMean       = mean(x);
    xVariance   = var(x);
    
    % Determine y-axis offset and variance
    y = [accelerometerZup(:,3); accelerometerXup(:,3)];
    yMean       = mean(y);
    yVariance   = var(y);
    
    % Determine z-axis offset and variance
    z = [accelerometerXup(:,4); accelerometerZleft(:,4)];
    zMean       = mean(z);
    zVariance   = var(z);
    
    % Prepare packages
    means = [xMean, yMean, zMean];
    variances = [xVariance, yVariance, zVariance];
    
    disp('Calculated axis offset: ');
    disp(num2str(means));
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));
    
    % Return data
    data = struct(...
        'offset', means, ...
        'variance', variances ...
        );
    
end