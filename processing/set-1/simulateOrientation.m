clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
%dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-z-pointing-up';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-x-pointing-forward';
dataSetFolder = '../../data/set-1/rotate-360ccw-around-y-pointing-left';

%% Load the data
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
    m = calibrateCompass(m);
      
    % Compensate for MPU6050 z axis sign
    % (Z axis on the MPU6050 is sign flipped so that it shows the up
    % vector)
    a = a .* [1, 1, -1];
    
    % Unfortunately, when reading the HMC5883L, the Y and Z readings
    % are swapped, so the order is X, Z, Y.
    m = [m(1), m(3), m(2)];

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
    end
    DCM = DCM*baseDCM;
    
    % plot the orientation
    plotOrientation(DCM, coordinateSystem, an, mn);
end