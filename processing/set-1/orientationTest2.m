close all; clear all; clc; home;

% 1) determine global Z axis using accelerometer
% 2) determine global X axis using magnetometer
% 3) determine global Y axis using cross of 1) and 2)
% 4) determine azimuth and elevation from magnetometer against 1,2,3)
% 5) determine roll using accelerometer against 1)

%% Define test vectors

% Roll test
control_roll = 20;
control_yaw = 10;
control_pitch = 90;

% vectors are rotated in yaw-pitch-roll order
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
cosPitchYsinRollX = -DCM(2,3);
cosPitchYcosRollX =  DCM(3,3);
rollX = -atan2(cosPitchYsinRollX, cosPitchYcosRollX) * 180/pi

cosPitchYcosYawZ =  DCM(1,1);
cosPitchYsinYawZ = -DCM(1,2);
yawZ = -atan2(cosPitchYsinYawZ, cosPitchYcosYawZ) * 180/pi

sinYawZ = sind(yawZ);
cosYawZ = cosd(yawZ);
sinPitchY = DCM(1,3);

%pitchY1 = -atan2(sinPitchY, cosPitchYsinYawZ/sinYawZ) * 180/pi
%pitchY2 = -atan2(sinPitchY*sinYawZ, cosPitchYsinYawZ) * 180/pi
pitchY3 = -atan2(sinPitchY, cosPitchYcosYawZ/cosYawZ) * 180/pi;
%pitchY4 = -atan2(sinPitchY*cosYawZ, cosPitchYcosYawZ) * 180/pi
%pitchY4 = -atan2(sinPitchY, cosPitchYsinRollX/sind(rollX)) * 180/pi
%pitchY5 = -atan2(sinPitchY*sind(rollX), cosPitchYsinRollX) * 180/pi
%pitchY6 = -atan2(sinPitchY, cosPitchYcosRollX/cosd(rollX)) * 180/pi
pitchY7 = -atan2(sinPitchY*cosd(rollX), cosPitchYcosRollX) * 180/pi;
if rollX >= 90 || rollX <= -90
    pitchY = pitchY3
else
    pitchY = pitchY7
end

azimuthd = yawZ;
elevationd = pitchY;
rolld = rollX;


%% Test calculated vectors

% Vectors need to be rotated in reverse order, that is roll-pitch-yaw
tx = rotateRPYd(xref, rolld, elevationd, azimuthd);
ty = rotateRPYd(yref, rolld, elevationd, azimuthd);
tz = rotateRPYd(zref, rolld, elevationd, azimuthd);

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