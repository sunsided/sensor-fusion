close all; clear all; clc; home;

% 1) determine global Z axis using accelerometer
% 2) determine global X axis using magnetometer
% 3) determine global Y axis using cross of 1) and 2)
% 4) determine azimuth and elevation from magnetometer against 1,2,3)
% 5) determine roll using accelerometer against 1)

%% Define test vectors

% Generic test
%{
a = [0; 1; 1];              % Accelerometer reading
m = [1; -0.25; 0.25];        % Magnetometer reading
%}

% Horizontal (azimuth) test
%{
a = [0; 0; 1];
m = [1; 0.5774; 0];
%}

% Vertical (heading) test
%{
a = [-0.5; 0; 1];
m = [1; 0; 0.5];
%}

% Roll test
a = [0; 0.5; 1];
m = [1; 0; 0];

%% Define local coordinate system

x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

%% Prepare measurements

an = a / norm(a);
mn = m / norm(m);

% define origin vector
O = [0; 0; 0];

% after normalisation, an (positive up) is the the Z axis
Z = an;             % Z is already normalised

% after normalisation, mn is identical to the X axis
X = mn;             % X is already normalised

% calculate global Y by crossing X and Z
Y = cross(Z, X);    % Y is already normalised

% For debugging purposes, re-generate X from Z and Y
%X = cross(Y,Z)

%% Calculate axis cross products

xX = cross(x, X)
xY = cross(x, Y)
xZ = cross(x, Z)

yX = cross(y, X)
yY = cross(y, Y)
yZ = cross(y, Z)

zX = cross(z, X)
zY = cross(z, Y)
zZ = cross(z, Z)

%% Calculate angles

azimuth = -atan2(xX(3),xY(3));
azimuthd = azimuth * 180/pi

heading = -atan2(yZ(3), zX(2));
headingd = heading * 180/pi

%% Plotting of the data

% Plot reference system 
figure;
ax = subplot(1, 1, 1);
plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], '-r', 'LineWidth', 3); hold on;
plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], '-g', 'LineWidth', 3);
plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], '-b', 'LineWidth', 3);

% Plot local coordinate system
plot3([O(1) x(1)], [O(2) x(2)], [O(3) x(3)], '-r', 'LineWidth', 1); hold on;
plot3([O(1) y(1)], [O(2) y(2)], [O(3) y(3)], '-g', 'LineWidth', 1);
plot3([O(1) z(1)], [O(2) z(2)], [O(3) z(3)], '-b', 'LineWidth', 1);

axis equal;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
grid on;

%set(ax, 'CameraUpVector', Z);
xlabel('forward');
ylabel('left');
zlabel('up');