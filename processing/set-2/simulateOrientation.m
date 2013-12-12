clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
%dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-z-pointing-up';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-x-pointing-forward';

%% Load the data
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

% resample the time series
[accelerometer, magnetometer] = lerpTimeSeries(accelerometer, magnetometer);

%% Prepare Plots
preparePlotOrientation();

%% Animation
baseDCM = [];
N = accelerometer.Length;
for n=1:2:N

    % Fetch sensor axes
    a = accelerometer.Data(n, :);
    m = magnetometer.Data(n, :);
   
    % Calibrate values
    a = calibrateAccelerometer(a);
    m = calibrateMagnetometer(m);
      
    % Normalize for later use
    an = a/norm(a);
    mn = m/norm(m);
    
    % Debugging
    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                    a(1), a(2), a(3), m(1), m(2), m(3));
    disp(msg);
    
    % Fetch rotation
    [~, ~, ~, DCM, coordinateSystem] = yawPitchRoll(a, m);

    % rotate relative to original rotation.
    if isempty(baseDCM)
        baseDCM = DCM';
    else
        DCM = DCM*baseDCM;
    end
    
    % plot the orientation
    plotOrientation(DCM, coordinateSystem, an, mn);
end