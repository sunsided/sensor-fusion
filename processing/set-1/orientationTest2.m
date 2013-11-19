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
control_roll = 45;
control_yaw = 90;
control_pitch = 45;

xref = rotateYPRd([1; 0; 0], control_roll, control_pitch, control_yaw);
yref = rotateYPRd([0; 1; 0], control_roll, control_pitch, control_yaw);
zref = rotateYPRd([0; 0; 1], control_roll, control_pitch, control_yaw);

a = zref;
m = xref;

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

% re-generate X from Z and Y
X = cross(Y,Z);     % Y is normalised because of Z and Y

% generate direction cosine matrix.
% this matrix contains all the rotation angles that have been applied
% beforehand by the rotate() method
DCM = [ ...
    dot(X, x),  dot(Y, x),  dot(Z, x);
    dot(X, y),  dot(Y, y),  dot(Z, y);
    dot(X, z),  dot(Y, z),  dot(Z, z);
    ]

% extract angles
cosPitchYsinRollX = DCM(3,2);
cosPitchYcosRollX = DCM(3,3);
rollX = -atan2(cosPitchYsinRollX, cosPitchYcosRollX) * 180/pi

cosPitchYcosYawZ = DCM(1,1);
cosPitchYsinYawZ = DCM(2,1);
yawZ = -atan2(cosPitchYsinYawZ, cosPitchYcosYawZ) * 180/pi

sinYawZ = sind(yawZ);
cosYawZ = cosd(yawZ);
sinPitchY = -DCM(3,1);
%pitchY = -atan2(sinPitchY, cosPitchYsinYawZ/sinYawZ) * 180/pi
%pitchY = -atan2(sinPitchY*sinYawZ, cosPitchYsinYawZ) * 180/pi
%pitchY = -atan2(sinPitchY, cosPitchYcosYawZ/cosYawZ) * 180/pi
pitchY = -atan2(sinPitchY*cosYawZ, cosPitchYcosYawZ) * 180/pi

azimuthd = yawZ;
elevationd = pitchY;
rolld = rollX;

%{
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

control_yaw
control_pitch
control_roll

Xp = [X(1) X(2) 0]; % project X into X/Y plane
%cXx = cross(X,x)
%cYz = cross(Y,z)
%azimuth = -atan2(Xp(2), Xp(1));
%azimuth = -atan2(cYz(2), cYz(1));
azimuth = -atan2(Xp(2), Xp(1));
azimuthd = round(azimuth * 180/pi * 1000) / 1000

% unfortunately these coordinates are the rotated ones, so axis swapping
% occurs
cZx = cross(Z,x); % project Z into X/Z plane
elevation = -atan2(cZx(3), cZx(2))
elevationd = round(elevation * 180/pi * 1000) / 1000

Yp = [0 Y(2) Y(3)]; % project Y into Y/Z plane
roll = -atan2(Yp(3), Yp(2));
rolld = round(roll * 180/pi * 1000) / 1000

%}

%% Test calculated vectors

tx = rotateYPRd(xref, rolld, elevationd, azimuthd);
ty = rotateYPRd(yref, rolld, elevationd, azimuthd);
tz = rotateYPRd(zref, rolld, elevationd, azimuthd);

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

% Plot test coordinate system
plot3([O(1) tx(1)], [O(2) tx(2)], [O(3) tx(3)], '--r', 'LineWidth', 2); hold on;
plot3([O(1) ty(1)], [O(2) ty(2)], [O(3) ty(3)], '--g', 'LineWidth', 2);
plot3([O(1) tz(1)], [O(2) tz(2)], [O(3) tz(3)], '--b', 'LineWidth', 2);

axis equal;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
%grid on;

%set(ax, 'CameraUpVector', Z);
xlabel('forward');
ylabel('left');
zlabel('up');
rotate3d