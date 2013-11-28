%clear all; close all; clc; home;

% define the data set folder
dataSet = 'rotate-360ccw-around-y-pointing-left';
dataSetFolder = fullfile(fileparts(which(mfilename)), '..', '..' , '..', 'data', 'set-1', dataSet);

% add parent folder to path
path(fullfile(fileparts(which(mfilename)), '..'), path);

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);
N = min(size(accelerometer,1), size(magnetometer,1));

%% Prepare Plots
preparePlotTrajectory(dataSet);

% Prepare angles
angles = NaN(N,3);

%% Animation
for n=1:N

    % Fetch accelerometer axes
    a = accelerometer(n, 2:4);
    
    % Fetch magnetometer axes
    m = [magnetometer(n, 2);
         magnetometer(n, 3);
         magnetometer(n, 4)]';
    
    % Calibrate values
    a = calibrateAccelerometer(a);
    m = calibrateCompass(m);
    
    % Unfortunately, when reading the HMC5883L, the Y and Z readings
    % are swapped, so the order is X, Z, Y.
    % Also, for the Drotek breakout, positive Z is pointing down.
    m = [m(1); m(3); -m(2)];
    
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
        theta_Mz = theta_Mz + 360
    end
    
    theta_My = atan2(m(1), -m(3)) * 180/pi; % pitch, attitude, elevation, nicken
    if theta_My < 0
        theta_My = theta_My + 360
    end
    
    theta_Mx = atan2(m(2), -m(3)) * 180/pi; % roll
    if theta_Mx < 0
        theta_Mx = theta_Mx + 360
    end
    
    msg = sprintf('theta: %+1.3f %+1.3f %+1.3f', ... 
                    theta_Mx, theta_My, theta_Mz);
    disp(msg);
    angles(n,:) = [theta_Mx, theta_My, theta_Mz];
        
    % plot the orientation
    if mod(n,10) == 0
        plotTrajectory(m);
    end
end

figure('Name', ['Axes: ' dataSet], 'NumberTitle', 'off');
subplot(3,1,1);
plot(angles(:,1));
ylim([0 360]);
xlabel('t'); ylabel('heading (tMx)');
subplot(3,1,2);
plot(angles(:,2));
ylim([0 360]);
xlabel('t'); ylabel('theta y');
subplot(3,1,3);
plot(angles(:,3));
ylim([0 360]);
xlabel('t'); ylabel('theta z');