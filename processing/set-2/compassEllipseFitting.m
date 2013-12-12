% define the data set folder
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'full-sphere');

%% Load the data
% note that this also works for accelerometer calibration
[~, ~, magnetometer, temperature] = loadData(dataSetFolder);

% Fetch axes
x = magnetometer.Data(:, 1);
y = magnetometer.Data(:, 2);
z = magnetometer.Data(:, 3);

% Calibrate sensor
path(fullfile(fileparts(which(mfilename)), 'calibration'), path);
calibrateByEllipseFitting(x, y, z);
