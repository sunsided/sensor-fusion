clc; clear all; home;

% define the data set folder
dataSetFolder = '../../data/set-1/tilt-sphere';

%% Load the data
[accelerometer, ~, ~, ~] = loadData(dataSetFolder);

% Fetch axes
x = accelerometer(:, 2);
y = accelerometer(:, 3);
z = accelerometer(:, 4);

% Calibrate sensor
calibrateByEllipseFitting(x, y, z);
