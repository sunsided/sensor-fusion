clc; clear all; home;

% define the data set folder
dataSetFolder = '../../data/set-1/tilt-sphere';

%% Load the data
% note that this also works for accelerometer calibration
[~, ~, magnetometer, temperature] = loadData(dataSetFolder);

% Fetch axes
x = magnetometer(:, 2);
y = magnetometer(:, 3);
z = magnetometer(:, 4);

% Calibrate sensor
calibrateByEllipseFitting(x, y, z);
