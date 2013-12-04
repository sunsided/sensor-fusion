%clear all; close all; clc; home;

% define the data set folder
dataSet = 'rotate-ccw-around-x-pointing-forward';
dataSetFolder = fullfile(fileparts(which(mfilename)), '..', '..' , '..', 'data', 'set-2', dataSet);

% add parent folder to path
path(fullfile(fileparts(which(mfilename)), '..'), path);

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

% resample the time series
[accelerometer, magnetometer] = lerpTimeSeries(accelerometer, magnetometer);
N = accelerometer.Length;

%% Prepare Plots
preparePlotTrajectory(dataSet);

% Prepare angles
angles1 = NaN(N,3);
angles2 = NaN(N,3);

%% Animation
for n=1:1:N

    % Fetch accelerometer axes
    a = accelerometer.Data(n, :);
    
    % Fetch magnetometer axes
    m = magnetometer.Data(n, :);
    
    % Calibrate values
    a = calibrateAccelerometer(a);
    m = calibrateCompass(m);
        
    % Normalize for later use
    an = a/norm(a);
    mn = m/norm(m);
    
    % Debugging
    %{
    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                    a(1), a(2), a(3), m(1), m(2), m(3));
    disp(msg);
    %}
    
    %theta_Mx = atan2(m(2), m(1)) * 180/pi;
    %theta_My = atan2(m(3), m(2)) * 180/pi;
    %theta_Mz = atan2(m(1), m(3)) * 180/pi;

    % following code taken from webbot sources, (c) 2011 Clive Webster
    theta_Mz = atan2(m(1), -m(2)) * 180/pi; % bearing, yaw, heading, gieren
    if theta_Mz < 0
        theta_Mz = theta_Mz + 360;
    end
    
    theta_My = atan2(m(1), -m(3)) * 180/pi; % pitch, attitude, elevation, nicken
    if theta_My < 0
        theta_My = theta_My + 360;
    end
    
    theta_Mx = atan2(m(2), -m(3)) * 180/pi; % roll
    if theta_Mx < 0
        theta_Mx = theta_Mx + 360;
    end
    
    msg = sprintf('theta mag: %+1.3f %+1.3f %+1.3f', ... 
                    theta_Mx, theta_My, theta_Mz);
    disp(msg);
    angles1(n,:) = [theta_Mx, theta_My, theta_Mz];
        
    
    
    % following code taken from webbot sources, (c) 2011 Clive Webster
    theta_Mz = atan2(a(1), -a(2)) * 180/pi; % bearing, yaw, heading, gieren
    if theta_Mz < -10
        theta_Mz = theta_Mz + 360;
    end
    
    theta_My = atan2(a(1), -a(3)) * 180/pi; % pitch, attitude, elevation, nicken
    if theta_My < -10
        theta_My = theta_My + 360;
    end
    
    theta_Mx = atan2(a(2), -a(3)) * 180/pi; % roll
    if theta_Mx < -10
        theta_Mx = theta_Mx + 360;
    end
    
    msg = sprintf('theta acc: %+1.3f %+1.3f %+1.3f', ... 
                    theta_Mx, theta_My, theta_Mz);
    disp(msg);
    angles2(n,:) = [theta_Mx, theta_My, theta_Mz];
    disp(' ');
       
    
    
    
    
    % plot the orientation
    if mod(n,30) == 0
        plotTrajectory(m);
    end
end

figure('Name', ['Axes: ' dataSet], 'NumberTitle', 'off');

subplot(3,1,1);
plot(angles1(:,1), ':', 'Color', [0.5 0.5 1]); hold on;
plot(angles2(:,1), 'k+', 'MarkerSize', 4);
ylim([0 360]);
xlabel('t'); ylabel('roll (\theta M_z)');

subplot(3,1,2);
plot(angles1(:,2)); hold on;
plot(angles2(:,2), 'k+', 'MarkerSize', 4); hold on;
ylim([0 360]);
xlabel('t'); ylabel('pitch (\theta M_y)');

subplot(3,1,3);
plot(angles1(:,3)); hold on;
plot(angles2(:,3), 'k+', 'MarkerSize', 4, 'Color', [0.5 0.5 0.5]);
ylim([0 360]);
xlabel('t'); ylabel('heading (\theta M_x)');