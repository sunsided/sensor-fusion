clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

%% Prepare Plots
preparePlotOrientation();

%% Animation
N = min(size(accelerometer,1), size(magnetometer,1));
for n=1:N

    % Fetch accelerometer axes
    a = accelerometer(n, 2:4);
    
    % Fetch magnetometer axes
    m = [-magnetometer(n, 3);
         -magnetometer(n, 2);
          magnetometer(n, 4)]';
    
    % Calibrate values
    a = calibrateAccelerometer(a);
    m = calibrateCompass(m);
      
    % Normalize for later use
    an = a/norm(a);
    mn = m/norm(m);
          
    % Debugging
    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                    a(1), a(2), a(3), m(1), m(2), m(3));
    disp(msg);
    
    % Fetch rotation
    [~, ~, ~, DCM, coordinateSystem] = yawPitchRoll(a, m);
    
    % plot the orientation
    plotOrientation(DCM, coordinateSystem, an, mn);
end