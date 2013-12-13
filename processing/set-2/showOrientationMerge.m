clear all; home;

%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-forward');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-y-pointing-left');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
[accelerometer, ~, compass, ~] = loadData(dataSetFolder, true);

% resample the time series
[acceleration, compass] = lerpTimeSeries(accelerometer, compass);

% extract time vector
time = acceleration.Time;
N = acceleration.Length;

%% Get roll, pitch and yaw
hwb = waitbar(0, 'Calculating states ...');

ypr = zeros(N, 3);
ypr2 = zeros(N, 3);
ypr3 = zeros(N, 3);

oldDCM = zeros(3);
oldOrientation = zeros(1,3);

for i=1:N   
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

    % clamp the angular velocity to +/-180 degree
    if ypr2(i,1) > 180
        ypr2(i,1) = ypr2(i,1) - 360;
    elseif ypr2(i,1) < -180
        ypr2(i,1) = ypr2(i,1) + 360;
    end
    
    if ypr2(i,2) > 180
        ypr2(i,2) = ypr2(i,2) - 360;
    elseif ypr2(i,2) < -180
        ypr2(i,2) = ypr2(i,2) + 360;
    end
    
    if ypr2(i,3) > 180
        ypr2(i,3) = ypr2(i,3) - 360;
    elseif ypr2(i,3) < -180
        ypr2(i,3) = ypr2(i,3) + 360;
    end
        
    % save current DCM for next iteration
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
    
    %{
    x = x/norm(x);
    y = y/norm(y);
    z = z/norm(z);
    %}
    
    
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
    
    om_pitchY = -asind(DCM(1, 3));
    om_rollX = atan2d(DCM(2, 3), DCM(3, 3));
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

    %{
    % clamp the angular velocity to +/-180 degree
    if ypr3(i,1) > 180
        ypr3(i,1) = ypr3(i,1) - 360;
    elseif ypr3(i,1) < -180
        ypr3(i,1) = ypr3(i,1) + 360;
    end
    
    if ypr3(i,2) > 180
        ypr3(i,2) = ypr3(i,2) - 360;
    elseif ypr3(i,2) < -180
        ypr3(i,2) = ypr3(i,2) + 360;
    end
    
    if ypr3(i,3) > 180
        ypr3(i,3) = ypr3(i,3) - 360;
    elseif ypr3(i,3) < -180
        ypr3(i,3) = ypr3(i,3) + 360;
    end
    %}
    
    oldOrientation = coordinateSystem;
    
    waitbar(i/N, hwb);
end
close(hwb);

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

%% Acceleration
% acceleration: x axis
axisAccel(1) = subplot(3, 4, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
x = acceleration.Data(:, 1);
line(t, x, ...
    'Parent', axisAccel(1), ...
    'Color', lineColor(1, :) ...
    ); 

xlim([0 t(end)]);
ylim([-2 2]);

title('Acceleration', ...
    'Color', titleColor ...
    );
ylabel('a_x [g]');
xlabel('t [s]');

% acceleration: y axis
axisAccel(2) = subplot(3, 4, 5, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
y = acceleration.Data(:, 2);
line(t, y, ...
    'Parent', axisAccel(2), ...
    'Color', lineColor(2, :) ...
    );

xlim([0 t(end)]);
ylim([-2 2]);

ylabel('a_y [g]');
xlabel('t [s]');

% acceleration: z axis
axisAccel(3) = subplot(3, 4, 9, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
z = acceleration.Data(:, 3);
line(t, z, ...
    'Parent', axisAccel(3), ...
    'Color', lineColor(3, :) ...
    );

xlim([0 t(end)]);
ylim([-2 2]);

ylabel('a_z [g]');
xlabel('t [s]');

%% Magnetometer
% compass: x axis
axisCompass(1) = subplot(3, 4, 2, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
x = compass.Data(:, 1);
line(t, x, ...
    'Parent', axisCompass(1), ...
    'Color', lineColor(1, :) ...
    ); 

xlim([0 t(end)]);
ylim([-2 2]);

title('Magnetometer', ...
    'Color', titleColor ...
    );
ylabel('a_x [g]');
xlabel('t [s]');

% compass: y axis
axisCompass(2) = subplot(3, 4, 6, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
y = compass.Data(:, 2);
line(t, y, ...
    'Parent', axisCompass(2), ...
    'Color', lineColor(2, :) ...
    );

xlim([0 t(end)]);
ylim([-2 2]);

ylabel('a_y [g]');
xlabel('t [s]');

% compass: z axis
axisCompass(3) = subplot(3, 4, 10, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
z = compass.Data(:, 3);
line(t, z, ...
    'Parent', axisCompass(3), ...
    'Color', lineColor(3, :) ...
    );

xlim([0 t(end)]);
ylim([-2 2]);

ylabel('a_z [g]');
xlabel('t [s]');


%% Roll
axisRpy(1) = subplot(3, 4, 3:4, ...
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
ylabel('angle [�]');
xlabel('t [s]');
legendHandle = legend('roll');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(2) = subplot(3, 4, 7:8, ...
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
ylabel('angle [�]');
xlabel('t [s]');
legendHandle = legend('pitch');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(3) = subplot(3, 4, 11:12, ...
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
yaw = ypr3(:, 3);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 0 0] ...
    ); 


xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth)', ...
    'Color', titleColor ...
    );
ylabel('angle [�]');
xlabel('t [s]');
legendHandle = legend('yaw');
set(legendHandle, 'TextColor', [1 1 1]);

%% Plot refining
% link the axes
linkaxes([axisAccel axisCompass axisRpy], 'x');
%linkaxes(axisRpy, 'x');