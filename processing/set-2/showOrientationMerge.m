clear all; home;

%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
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

%% Get roll, pitch and yaw
hwb = waitbar(0, 'Calculating states ...');

ypr = zeros(N, 3);
ypr2 = zeros(N, 3);
ypr3 = zeros(N, 3);
ypr_gyro = zeros(N, 3);

oldDCM = zeros(3);
oldOrientation = zeros(1,3);

for i=1:N
    % fetch RPY from integrated gyro
    ypr_gyro(i, :) = [0, 0, 0];    
    if i > 1
        ypr_gyro_current = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        dt = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + ypr_gyro_current * dt;
    end
    
    % fetch RPY from accelerometer and magnetometer
    a = acceleration.Data(i, :);
    m = compass.Data(i, :);
    [yaw, pitch, roll, DCM, coordinateSystem, ~] = yawPitchRoll(a, m);
    ypr(i,:) = [yaw, pitch, roll];
    
    % calculate the difference of the rotation and hence the 
    % angular velocity
    difference = DCM'*oldDCM;   
    om_pitchY = -asind(difference(1, 3));
    om_rollX = atan2d(difference(2, 3), difference(3, 3));
    om_yawZ = atan2d(difference(1, 2), difference(1, 1));
    
    % integrate the angular velocity
    old_ypr2 = ypr(i, :);
    if i > 1
        old_ypr2 = ypr2(i-1, :);
    end
    ypr2(i, :) = old_ypr2 + [om_yawZ, om_pitchY, om_rollX];
        
    % save current DCM for next iteration
    diffDCM = difference;
    oldDCM = DCM;
    
    
    
    % integrate the angular velocity
    if i == 1
        oldOrientation = coordinateSystem;
    end
    difference = coordinateSystem - oldOrientation;
    X = [1 0 0];
    Y = [0 1 0];
    Z = [0 0 1];
    
    x = difference(1,:) + X;
    y = difference(2,:) + Y;
    z = difference(3,:) + Z;
    
    x = x/norm(x);
    y = y/norm(y);
    z = z/norm(z);    
    
    DCM = [ ...
        dot(x, X),  dot(y, X),  dot(z, X);
        dot(x, Y),  dot(y, Y),  dot(z, Y);
        dot(x, Z),  dot(y, Z),  dot(z, Z);
        ];
    
    
    %{
    om_yawZ   = asind( dot(cross(x, [1 0 0]), [0 0 1]));
    om_rollX  = asind( dot(cross(y, [0 1 0]), [1 0 0]));
    om_pitchY = asind( dot(cross(x, [1 0 0]), [0 1 0]));
    %}
    %{
    om_yawZ   = asind( 0.5 * (dot(cross(x, [1 0 0]), [0 0 1]) + dot(cross(y, [0 1 0]), [0 0 1])));
    om_rollX  = asind( 0.5 * (dot(cross(y, [0 1 0]), [1 0 0]) + dot(cross(z, [0 0 1]), [1 0 0])));
    om_pitchY = asind( 0.5 * (dot(cross(x, [1 0 0]), [0 1 0]) + dot(cross(z, [0 0 1]), [0 1 0])));
    %}
    %{
    om_yawZ   = atan2(  dot(cross(x, [0 1 0]), [0 0 1]), dot(cross(x, [1 0 0]), [0 0 1]));
    om_rollX  = asind( 0.5 * (dot(cross(y, [0 1 0]), [1 0 0]) + dot(cross(z, [0 0 1]), [1 0 0])));
    om_pitchY = asind( 0.5 * (dot(cross(x, [1 0 0]), [0 1 0]) + dot(cross(z, [0 0 1]), [0 1 0])));
    %}

    om_rollX = atan2d(DCM(2, 3), DCM(3, 3));
    om_pitchY = -asind(DCM(1, 3));
    om_yawZ = atan2d(DCM(1, 2), DCM(1, 1));
    

    %{
    om_pitchY = -asind(dot(z, X));
    om_rollX = atan2d(dot(z, Y), dot(z, Z));
    om_yawZ = atan2d(dot(y, X), dot(x, X));
    %}
    
    % integrate the angular velocity
    old_ypr3 = ypr(i, :);
    if i > 1
        old_ypr3 = ypr3(i-1, :);
    end
    ypr3(i, :) = old_ypr3 + [om_yawZ, om_pitchY, om_rollX];
    
    oldOrientation = coordinateSystem;
    
    waitbar(i/N, hwb);
end
close(hwb);

% clamp angles to -180..180
ypr2 = clampangle(ypr2);
ypr_gyro = clampangle(ypr_gyro);

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
roll = ypr2(:, 3);
line(t, roll, ...
    'Parent', axisRpy(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 
roll = ypr3(:, 3);
line(t, roll, ...
    'Parent', axisRpy(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 0 0] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int', 'axis int');
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
pitch = ypr2(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(2), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 
pitch = ypr3(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(2), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 0 0] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int', 'axis int');
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
yaw = ypr2(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 
yaw = ypr3(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 0 0] ...
    ); 


xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int', 'axis int');
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

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int');
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

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int');
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


xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int');
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
roll = -gyroscope.Data(:,1);
line(t, roll, ...
    'Parent', axisRpy(7), ...
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
legendHandle = legend('gyro');
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
pitch = -gyroscope.Data(:,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
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
legendHandle = legend('gyro');
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
yaw = gyroscope.Data(:,3);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
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
legendHandle = legend('gyro');
set(legendHandle, 'TextColor', [1 1 1]);



%% Plot refining
% link the axes
linkaxes(axisRpy, 'x');