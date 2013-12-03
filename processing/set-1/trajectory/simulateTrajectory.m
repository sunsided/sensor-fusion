%clear all; close all; clc; home;

% define the data set folder
dataSet = 'rotate-360ccw-around-x-pointing-forward';
dataSetFolder = fullfile(fileparts(which(mfilename)), '..', '..' , '..', 'data', 'set-1', dataSet);

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
    
    % A problem with the MPU6050 is that the registers do containt the
    % strongest acceleration with z sign flipped. This is good as one
    % is usually interested in the up vector, but it can result in a case
    % where up and magnetic vector are perpendicular to each other.
    % Thus for calculation of the coordinate system we will flip the Z
    % sign.
    a = [a(1); a(2); -a(3)];
    
    % Unfortunately, when reading the HMC5883L, the Y and Z readings
    % are swapped, so the order is X, Z, Y.
    %
    % NOTE: From a quick look it seems that the Z axis sign is flipped
    %       when in reality it is the perception of the magnetic field
    %       that is wrong. The magnetic field is only perpendicular to
    %       the earth's surface when at the equator. On the northern
    %       hemisphere, the magnetic field enters earth at an angle
    %       (a downward angled vector!) while at the southern hemisphere
    %       it shoots out of earth at an angle.
    % see: http://forum.arduino.cc/index.php?topic=152529.msg1147915#msg1147915
    %      http://forum.arduino.cc/index.php?topic=152529.msg1150084#msg1150084
    m = [m(1); m(3); m(2)];
    
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