%% Load data

clear
clc

% Fetch all data.
load('hmc5883l_compass.mat');
load('mpu6050_accelerometer.mat');
load('mpu6050_gyroscope.mat');

% Find and remove rows where all elements are zero
accelBuffer(all(accelBuffer == 0, 2), :) = [];
gyroBuffer(all(gyroBuffer == 0, 2), :) = [];
compassBuffer(all(compassBuffer == 0, 2), :) = [];

close all;

colors = copper;

figure;

%% Accelerometer
axisMin = min(min(accelBuffer(:, 2:end)));
axisMax = max(max(accelBuffer(:, 3:end)));
axisLimit = max(abs([axisMin, axisMax])) * 1.25;
sizes = ones(size(accelBuffer(:, 1))) * 10;

% 3D Plot
subplot(3, 4, 1);
scatter3(accelBuffer(:, 2), accelBuffer(:, 3), accelBuffer(:, 4), sizes, accelBuffer(:, 1), 'filled');
colormap(colors);
title('Accelerometer 3D');
xlabel('X (g)');  % 1g = 9.81 m/s^2
ylabel('Y (g)');  % 1g = 9.81 m/s^2
zlabel('Z (g)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);
zlim([-axisLimit, axisLimit]);

% X/Y Plot
subplot(3, 4, 2);
scatter(accelBuffer(:, 2), accelBuffer(:, 3), sizes, accelBuffer(:, 1), 'filled');
colormap(colors);
title('Accelerometer X/Y');
xlabel('X (g)');  % 1g = 9.81 m/s^2
ylabel('Y (g)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% Y/Z Plot
subplot(3, 4, 3);
scatter(accelBuffer(:, 3), accelBuffer(:, 4), sizes, accelBuffer(:, 1), 'filled');
colormap(colors);
title('Accelerometer Y/Z');
xlabel('Y (g)');  % 1g = 9.81 m/s^2
ylabel('Z (g)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% X/Z Plot
subplot(3, 4, 4);
scatter(accelBuffer(:, 2), accelBuffer(:, 4), sizes, accelBuffer(:, 1), 'filled');
colormap(colors);
title('Accelerometer X/Z');
xlabel('X (g)');  % 1g = 9.81 m/s^2
ylabel('Z (g)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

%% Gyroscope
axisMin = min(min(gyroBuffer(:, 2:end)));
axisMax = max(max(gyroBuffer(:, 3:end)));
axisLimit = max(abs([axisMin, axisMax])) * 1.25;
sizes = ones(size(gyroBuffer(:, 1))) * 10;

% 3D Plot
subplot(3, 4, 5);
scatter3(gyroBuffer(:, 2), gyroBuffer(:, 3), gyroBuffer(:, 4), sizes, gyroBuffer(:, 1), 'filled');
colormap(colors);
title('Gyroscope 3D');
xlabel('X rad/s)');  % 1g = 9.81 m/s^2
ylabel('Y (rad/s)');  % 1g = 9.81 m/s^2
zlabel('Z (rad/s)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);
zlim([-axisLimit, axisLimit]);

% X/Y Plot
subplot(3, 4, 6);
scatter(gyroBuffer(:, 2), gyroBuffer(:, 3), sizes, gyroBuffer(:, 1), 'filled');
colormap(colors);
title('Gyroscope X/Y');
xlabel('X (rad/s)');  % 1g = 9.81 m/s^2
ylabel('Y (rad/s)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% Y/Z Plot
subplot(3, 4, 7);
scatter(gyroBuffer(:, 3), gyroBuffer(:, 4), sizes, gyroBuffer(:, 1), 'filled');
colormap(colors);
title('Gyroscope Y/Z');
xlabel('Y (rad/s)');  % 1g = 9.81 m/s^2
ylabel('Z (rad/s)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% X/Z Plot
subplot(3, 4, 8);
scatter(gyroBuffer(:, 2), gyroBuffer(:, 4), sizes, gyroBuffer(:, 1), 'filled');
colormap(colors);
title('Gyroscope X/Z');
xlabel('X (rad/s)');  % 1g = 9.81 m/s^2
ylabel('Z (rad/s)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);


%% Magnetometer
axisMin = min(min(compassBuffer(:, 2:end)));
axisMax = max(max(compassBuffer(:, 3:end)));
axisLimit = max(abs([axisMin, axisMax])) * 1.25;
sizes = ones(size(compassBuffer(:, 1))) * 10;

% 3D Plot
subplot(3, 4, 9);
scatter3(compassBuffer(:, 2), compassBuffer(:, 3), compassBuffer(:, 4), sizes, compassBuffer(:, 1), 'filled');
colormap(colors);
title('Magnetometer 3D');
xlabel('X (mG)');  % 1g = 9.81 m/s^2
ylabel('Y (mG)');  % 1g = 9.81 m/s^2
zlabel('Z (mG)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);
zlim([-axisLimit, axisLimit]);

% X/Y Plot
subplot(3, 4, 10);
scatter(compassBuffer(:, 2), compassBuffer(:, 3), sizes, compassBuffer(:, 1), 'filled');
colormap(colors);
title('Magnetometer X/Y');
xlabel('X (mG)');  % 1g = 9.81 m/s^2
ylabel('Y (mG)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% Y/Z Plot
subplot(3, 4, 11);
scatter(compassBuffer(:, 3), compassBuffer(:, 4), sizes, compassBuffer(:, 1), 'filled');
colormap(colors);
title('Magnetometer Y/Z');
xlabel('Y (mG)');  % 1g = 9.81 m/s^2
ylabel('Z (mG)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

% X/Z Plot
subplot(3, 4, 12);
scatter(compassBuffer(:, 2), compassBuffer(:, 4), sizes, compassBuffer(:, 1), 'filled');
colormap(colors);
title('Magnetometer X/Z');
xlabel('X (mG)');  % 1g = 9.81 m/s^2
ylabel('Z (mG)');  % 1g = 9.81 m/s^2
grid on;
axis square;
xlim([-axisLimit, axisLimit]);
ylim([-axisLimit, axisLimit]);

%% Save the plot

% Adjust figure properties
sgtitle('Sensor Readings (3D)');
set(gcf, 'Position', [100, 100, 1080, 940]);
saveas(gcf, 'sensor-readings-3d.png');

% close all;
