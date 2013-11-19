clear all; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';
dataSetFolder = '../../data/set-1/tilt-around-y-pointing-left';
%dataSetFolder = '../../data/set-1/tilt-around-z-pointing-up';

%% Load the data
[accelerometer, ~, compass, ~] = loadData(dataSetFolder);

% Calibrate data
accelerometer = calibrateAccelerometer(accelerometer);
compass = calibrateCompass(compass);

% Extract data for plotting
timeAcceleration = accelerometer(:,1);
acceleration = accelerometer(:,2:4);
timeCompass = compass(:,1);
compass = compass(:, [2 4 3]); % error in data, sensor axes are swapped

N = min(length(timeAcceleration), length(timeCompass));
time = timeAcceleration(1:N);

%% Get roll, pitch and yaw

ypr = zeros(N, 3);
for i=1:N
    a = acceleration(i, :);
    m = compass(i, :);
    [yaw pitch roll] = yawPitchRoll(a, m);
    ypr(i, :) = [yaw pitch roll];
end

% correct yaw for modulo breaks
yawDiff = diff(ypr(:,1));
indices = find(yawDiff < -90);
yawDiff(indices) = yawDiff(indices) + 360;
indices = find(yawDiff > 90);
yawDiff(indices) = yawDiff(indices) - 360;
ypr(:,1) = [0; cumsum(yawDiff(:))];

% correct pitch for modulo breaks
pitchDiff = diff(ypr(:,2));
indices = find(pitchDiff < -90);
pitchDiff(indices) = pitchDiff(indices) + 360;
indices = find(pitchDiff > 90);
pitchDiff(indices) = pitchDiff(indices) - 360;
ypr(:,2) = [0; cumsum(pitchDiff(:))];

% correct rool for modulo breaks
rollDiff = diff(ypr(:,3));
indices = find(rollDiff < -90);
rollDiff(indices) = rollDiff(indices) + 360;
indices = find(rollDiff > 90);
rollDiff(indices) = rollDiff(indices) - 360;
ypr(:,3) = [0; cumsum(rollDiff(:))];

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

t = timeAcceleration;
x = acceleration(:, 1);
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

t = timeAcceleration;
y = acceleration(:, 2);
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

t = timeAcceleration;
z = acceleration(:, 3);
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

t = timeCompass;
x = compass(:, 1);
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

t = timeCompass;
y = compass(:, 2);
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

t = timeCompass;
z = compass(:, 3);
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
    'Color', lineColor(4, :) ...
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
    'Color', lineColor(5, :) ...
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
    'Color', lineColor(6, :) ...
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