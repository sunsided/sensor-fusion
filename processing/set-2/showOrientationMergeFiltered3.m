clear all; home;

%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-forward');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-y-pointing-left');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-z-pointing-up');
[accelerometer, gyroscope, compass, ~] = loadData(dataSetFolder, true);

% resample the time series
[acceleration, gyroscope, compass] = lerpTimeSeries(accelerometer, gyroscope, compass);

% extract time vector
time = acceleration.Time;
N = acceleration.Length;

N = min(N, 10000);
time = time(1:N);

%% Prepare the Kalman filter

% state vector
x = [0 0 0, ... % angles
     0 0 0, ... % angular velocities
     0 0 0]';   % angular accelerations

gv = gyroscope.UserData.variance;
 
% state covariance matrix
P = [
     2  0  0    0  0  0     0  0  0;
     0  2  0    0  0  0     0  0  0;
     0  0  2    0  0  0     0  0  0;

     0  0  0  gv(1) 0 0     0  0  0;
     0  0  0    0 gv(2) 0   0  0  0;
     0  0  0    0  0 gv(3)  0  0  0;

     0  0  0    0  0  0    .1  0  0;
     0  0  0    0  0  0     0 .1  0;
     0  0  0    0  0  0     0  0 .1];

B = eye(size(P));

t = 0.001;
s = 0.002;
m = 0.01;

% too large values in Q yield bad results
Q = [
     m  0  0    0  0  0     0  0  0;
     0  m  0    0  0  0     0  0  0;
     0  0  m    0  0  0     0  0  0;

     0  0  0    t  0  0     0  0  0;
     0  0  0    0  t  0     0  0  0;
     0  0  0    0  0  t     0  0  0;

     0  0  0    0  0  0     s  0  0;
     0  0  0    0  0  0     0  s  0;
     0  0  0    0  0  0     0  0  s];
   
% Lambda coefficient for artificial increase of covariance
lambda = .95;

%% Get roll, pitch and yaw
hwb = waitbar(0, 'Calculating states ...');

ypr = zeros(N, 3);
ypr2 = zeros(N, 3);
ypr_kf = zeros(N, 3);
ypr_gyro = zeros(N, 3);
omega_kf = zeros(N, 3);

oldDCM = zeros(3);
ypr_base = [0 0 0];
previous_ypr = [0 0 0];

for i=1:N
    % fetch RPY from accelerometer and magnetometer
    a = acceleration.Data(i, :);
    m = compass.Data(i, :);
    
    [yaw, pitch, roll, DCM, coordinateSystem, ~] = yawPitchRoll(a, m);
    
    current_ypr = [yaw, pitch, roll];
    ypr(i,:) = current_ypr;
    
    % get time derivative
    T = 0.1;
    ypr_gyro_current = [0 0 0];
    if i > 1
        T = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro_current = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + ypr_gyro_current * T;
    end
    
    % as for IDDCM ... Euler rates are not body rates.
    
    % Prepare Kalman Filter
   
    % state matrix
    A = [
         1 0 0    T 0 0     0.5*T^2 0 0;
         0 1 0    0 T 0     0 0.5*T^2 0;
         0 0 1    0 0 T     0 0 0.5*T^2;
         
         0 0 0    1 0 0     T 0 0;
         0 0 0    0 1 0     0 T 0;
         0 0 0    0 0 1     0 0 T;
         
         0 0 0    0 0 0     1 0 0;
         0 0 0    0 0 0     0 1 0;
         0 0 0    0 0 0     0 0 1];
    
    % Kalman Filter: Initial Prediction
    if i == 1
        previous_ypr = current_ypr;
        
        x(1) = yaw;       
        x(2) = pitch;
        x(3) = roll;

        [x, P] = kf_predict(x, A, P, lambda, Q);
    end
      
    % glitch correction
    %{
    for axis = 1:3
        diff = current_ypr(axis) - previous_ypr(axis);
        if diff < -100
            current_ypr(axis) = current_ypr(axis) + 360;
        end
        
        diff = current_ypr(axis) - previous_ypr(axis);
        if diff > 100
            current_ypr(axis) = current_ypr(axis) - 360;
        end
    end
    %}
    
    previous_ypr = current_ypr;
    
    %{
    for axis = 1:3
        if x(axis) > 180
            x(axis) = x(axis) - 360;
        elseif x(axis) < -180
            x(axis) = x(axis) + 360;
        end
    end    
    
    if gyroscope.Time(i) > 17.97
        1;
    end
    %}
    
    % Axis R base value
    RA = 1;
    SwitchThreshold = 15;
    SwitchScale = 1;
    
    if abs(pitch) > SwitchThreshold

        % measurement transformation matrix
        H = [
             0 1 0 0 0 0 0 0 0;
             0 0 0 1 0 0 0 0 0;
             0 0 0 0 1 0 0 0 0;
             0 0 0 0 0 1 0 0 0];

        % measurement noise matrix
        R = [
           RA   0   0   0;
            0   gv(1)   0   0;
            0   0   gv(2)  0;
            0   0   0   gv(3)];


        % Measurement vector
        z = [
             pitch;
             ypr_gyro_current(1);
             ypr_gyro_current(2);
             ypr_gyro_current(3)];
    else
        
        % measurement transformation matrix
        H = [
             1 0 0 0 0 0 0 0 0;
             0 1 0 0 0 0 0 0 0;
             0 0 1 0 0 0 0 0 0;
             0 0 0 1 0 0 0 0 0;
             0 0 0 0 1 0 0 0 0;
             0 0 0 0 0 1 0 0 0];

        s = ((abs(pitch)+1)/(SwitchThreshold+1))^2 * SwitchScale + RA;
        c = ((abs(pitch)+1)/(SwitchThreshold+1))^2 * SwitchScale;
         
        % measurement noise matrix
        R = [
            s   0   0   0   0   0;
            0  RA   0   0   0   0;
            0   0   s   0   0   0;
            0   0   0   gv(1)   0   0;
            0   0   0   0   gv(2)  0;
            0   0   0   0   0   gv(3)];
        
        % Measurement vector
        z = [
             yaw;
             pitch;
             roll;
             ypr_gyro_current(1);
             ypr_gyro_current(2);
             ypr_gyro_current(3)];        
     
    end
    
    % Kalman Filter: Measurement Update
    [x, P] = kf_update(x, z, P, H, R);
    
    % correct axes
    %{
    for axis = 1:3
        if x(axis) > 180
            x(axis) = x(axis) - 360;
        elseif x(axis) < -180
            x(axis) = x(axis) + 360;
        end
    end
    %}
    
    % store state
    ypr_kf(i, :)    = [x(1)  x(2)  x(3)];
    omega_kf(i, :)  = [x(4), x(5), x(6)];
    
    % Kalman Filter: Predict
    [x, P] = kf_predict(x, A, P, lambda, Q);
        
    waitbar(i/N, hwb);
end
close(hwb);

% clamp angles to -180..180
ypr2 = clampangle(ypr2);
ypr_gyro = clampangle(ypr_gyro);
ypr_kf = clampangle(ypr_kf);


%% Plot data
figureHandle = figure('Name', 'Raw and derived inertial sensor data', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

% define base colors
lineColor(1, :) = [1 0.25 0]; % x axis
lineColor(2, :) = [0.5 1 0]; % y axis
lineColor(3, :) = [0 0.5 1]; % z axis

lineColor(4, :) = [1 1 0]; % x axis
lineColor(5, :) = [1 0 1]; % y axis
lineColor(6, :) = [1.0 0.75 0.25]; % z axis

meanColor = [1 1 1];
axesColor = [0.473 0.473 0.473];
plotBackground = [0.15 0.15 0.15];
titleColor = [1 1 1];


%% Roll
axisRpy(1) = subplot(3, 3, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = ypr(:, 3);
line(t, roll, ...
    'Parent', axisRpy(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(2) = subplot(3, 3, 4, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = ypr(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(2), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(3) = subplot(3, 3, 7, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = ypr(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
set(legendHandle, 'TextColor', [1 1 1]);


%% Roll
axisRpy(4) = subplot(3, 3, 2, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = ypr_gyro(:, 3);
line(t, roll, ...
    'Parent', axisRpy(4), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    );  
hold on;
roll = ypr_kf(:, 3);
line(t, roll, ...
    'Parent', axisRpy(4), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(5) = subplot(3, 3, 5, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = ypr_gyro(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(5), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
hold on;
pitch = ypr_kf(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(5), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(6) = subplot(3, 3, 8, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = ypr_gyro(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(6), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
hold on;
yaw = ypr_kf(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(6), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);



%% Roll
axisRpy(7) = subplot(3, 3, 3, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = -gyroscope.Data(1:N,1);
line(t, roll, ...
    'Parent', axisRpy(7), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    ); 
hold on;
roll = omega_kf(1:N,3);
line(t, roll, ...
    'Parent', axisRpy(7), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(8) = subplot(3, 3, 6, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = -gyroscope.Data(1:N,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
hold on;
pitch = omega_kf(1:N,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(9) = subplot(3, 3, 9, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = gyroscope.Data(1:N,3);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
hold on;
yaw = omega_kf(1:N,1);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);



%% Plot refining
% link the axes
linkaxes(axisRpy, 'xy');