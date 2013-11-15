close all; clear all; clc; home;

% Define test vector
v = [1; 0.5; 1];

% Define reference vectors
O = [0; 0; 0];
X = [1; 0; 0];
Y = [0; 1; 0];
Z = [0; 0; 1];

%% Calculation of angles

% Test vector normalization
vn = v / norm(v);

disp(['Test vector:            ' mat2str(v)]);
disp(['Normalised test vector: ' mat2str(vn)]);

% Component vectors of v
vx = vn.*X;
vy = vn.*Y;
vz = vn.*Z;

% v-Reference Axis cross products
xy = cross(vx, Y);
zx = cross(vz, X);
yz = cross(vy, Z);

% Calculate angle to X in XY (azimuth)
clear dx dy dz;

dx = xy(3);
dz = yz(1);
azimuth = -atan2(dz, dx);
disp(['Azimuth (heading): ' num2str(azimuth*180/pi) '° (' num2str(azimuth) ')']);

% Calculate angle to X in XZ (elevation)
clear dx dy dz;

dx = xy(3);
dy = zx(2);
elevation = -atan2(dy, dx);
disp(['Elevation:         ' num2str(elevation*180/pi) '° (' num2str(elevation) ')']);

%% Plotting

% Prepare plot
figure('Name', 'Orientation vector test', 'NumberTitle', 'off');

% Plot reference system in XY
subplot(3, 4, 1);
plot([O(1) X(1)], [O(2) X(2)], '-r', 'LineWidth', 2); hold on;
plot([O(1) Y(1)], [O(2) Y(2)], '-g', 'LineWidth', 2);
axis equal;
xlim([-2 2]);
ylim([-2 2]);
xlabel('x (forward)');
ylabel('y (left)');
grid on;

% Plot test vectorin XY
plot([O(1) v(1)], [O(2) v(2)], '-', 'Color', [0.5 0.7 0.7], 'LineWidth', 2);




% Plot reference system in XZ
subplot(3, 4, 5);
plot([O(1) X(1)], [O(3) X(3)], '-r', 'LineWidth', 2); hold on;
plot([O(1) Z(1)], [O(3) Z(3)], '-b', 'LineWidth', 2);
axis equal;
xlim([-2 2]);
ylim([-2 2]);
xlabel('x (forward)');
ylabel('z (up)');
grid on;

% Plot test vector
plot([O(1) v(1)], [O(3) v(3)], '-', 'Color', [0.5 0.7 0.7], 'LineWidth', 2);




% Plot reference system in YZ
subplot(3, 4, 9);
plot([O(1) Y(2)], [O(2) Y(1)], '-g', 'LineWidth', 2); hold on;
plot([O(1) Z(1)], [O(3) Z(3)], '-b', 'LineWidth', 2);
axis equal;
xlim([-2 2]);
ylim([-2 2]);
xlabel('y (left)');
ylabel('z (up)');
grid on;

% Plot test vector
plot([O(1) v(2)], [O(2) v(1)], '-', 'Color', [0.5 0.7 0.7], 'LineWidth', 2);




% Plot reference system 
subplot(3, 4, [2:4; 6:8; 10:12]);
plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], '-r', 'LineWidth', 2); hold on;
plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], '-g', 'LineWidth', 2);
plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], '-b', 'LineWidth', 2);
axis equal;
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
xlabel('x (forward)');
ylabel('y (left)');
zlabel('z (up)');
grid on;

% Plot test vector
plot3([O(1) v(1)], [O(2) v(2)], [O(3) v(3)], '-', 'Color', [0.5 0.7 0.7], 'LineWidth', 2);
rotate3d