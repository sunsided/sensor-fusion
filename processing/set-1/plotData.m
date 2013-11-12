close all; clear all; home;

% define the data set folder
dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

%% Plot data
figureHandle = figure('Name', 'Raw sensor data', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

% define base colors
lineColor(1, :) = [1 0.25 0]; % x axis
lineColor(2, :) = [0.5 1 0]; % y axis
lineColor(3, :) = [0 0.5 1]; % z axis
axesColor = [0.473 0.473 0.473];
plotBackground = [0.15 0.15 0.15];
titleColor = [1 1 1];

%% Accelerometer
% accelerometer: x axis
axisAccel(1) = subplot(3, 3, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = accelerometer(:, 1);
x = accelerometer(:, 2);
line(t, x, ...
    'Parent', axisAccel(1), ...
    'Color', lineColor(1, :) ...
    );

xlim([0 t(end)]);

title('MPU6050 Accelerometer', ...
    'Color', titleColor ...
    );
ylabel('a_x [g]');
xlabel('t [s]');

% accelerometer: y axis
axisAccel(2) = subplot(3, 3, 4, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = accelerometer(:, 1);
y = accelerometer(:, 3);
line(t, y, ...
    'Parent', axisAccel(2), ...
    'Color', lineColor(2, :) ...
    );

xlim([0 t(end)]);

ylabel('a_y [g]');
xlabel('t [s]');

% accelerometer: z axis
axisAccel(3) = subplot(3, 3, 7, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = accelerometer(:, 1);
z = accelerometer(:, 4);
line(t, z, ...
    'Parent', axisAccel(3), ...
    'Color', lineColor(3, :) ...
    );

xlim([0 t(end)]);

ylabel('a_z [g]');
xlabel('t [s]');

%% Gyroscope
% gyroscope: x axis
axisGyro(1) = subplot(3, 3, 2, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = gyroscope(:, 1);
x = gyroscope(:, 2);
line(t, x, ...
    'Parent', axisGyro(1), ...
    'Color', lineColor(1, :) ...
    );

xlim([0 t(end)]);

title('MPU6050 Gyroscope', ...
    'Color', titleColor ...
    );
ylabel('\omega_x [\circ/s]');
xlabel('t [s]');

% gyroscope: y axis
axisGyro(2) = subplot(3, 3, 5, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = gyroscope(:, 1);
y = gyroscope(:, 3);
line(t, y, ...
    'Parent', axisGyro(2), ...
    'Color', lineColor(2, :) ...
    );

xlim([0 t(end)]);

ylabel('\omega_y [\circ/s]');
xlabel('t [s]');

% gyroscope: z axis
axisGyro(3) = subplot(3, 3, 8, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = gyroscope(:, 1);
z = gyroscope(:, 4);
line(t, z, ...
    'Parent', axisGyro(3), ...
    'Color', lineColor(3, :) ...
    );

xlim([0 t(end)]);

ylabel('\omega_z [\circ/s]');
xlabel('t [s]');

%% Compass
% compass: x axis
axisCompass(1) = subplot(3, 3, 3, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = magnetometer(:, 1);
x = magnetometer(:, 2);
line(t, x, ...
    'Parent', axisCompass(1), ...
    'Color', lineColor(1, :) ...
    );

xlim([0 t(end)]);

title('HMC5883L Magnetometer', ...
    'Color', titleColor ...
    );
ylabel('B_x [Gs]');
xlabel('t [s]');

% compass: y axis
axisCompass(2) = subplot(3, 3, 6, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = magnetometer(:, 1);
y = magnetometer(:, 3);
line(t, y, ...
    'Parent', axisCompass(2), ...
    'Color', lineColor(2, :) ...
    );

xlim([0 t(end)]);

ylabel('B_y [Gs]');
xlabel('t [s]');

% compass: z axis
axisCompass(3) = subplot(3, 3, 9, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = magnetometer(:, 1);
z = magnetometer(:, 4);
line(t, z, ...
    'Parent', axisCompass(3), ...
    'Color', lineColor(3, :) ...
    );

xlim([0 t(end)]);

ylabel('B_z [Gs]');
xlabel('t [s]');

%% Plot refining
% link the axes
linkaxes([axisAccel axisGyro axisCompass], 'x');