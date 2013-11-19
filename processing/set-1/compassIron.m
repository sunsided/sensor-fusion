clc; clear all; home;

% define the data set folder
dataSetFolder = '../../data/set-1/tilt-sphere';

%% Load the data
[~, ~, magnetometer, temperature] = loadData(dataSetFolder);

% Fetch axes
x = magnetometer(:, 2);
y = magnetometer(:, 3);
z = magnetometer(:, 4);

% Calculate hard-iron factor
% In theory, a simple average (mean(magnetometer(:, 2:4)))
% would do the trick, but that does not take into account the
% actual number of measurements taken. Averging the component-wise
% min and max values yields better results but is prone to errors
% due to outliers in the measurement.
hardIronOffset(1) = (max(x)+min(x))/2;
hardIronOffset(2) = (max(y)+min(y))/2;
hardIronOffset(3) = (max(z)+min(z))/2;

disp('Calculated hard iron offset: ');
disp(num2str(hardIronOffset));

% Calculate soft-iron factors
% A naive approach: measure all distances from the center point
% to each point in the point cloud. The largest and smallest distances
% are equal to the major and minor axis of the ellipse, respectively.
% Take the angles to these vectors and rotate to X/Y, then scale to
% unity. Repeat to find the third axis.
xc = x - hardIronOffset(1);
yc = y - hardIronOffset(2);
zc = z - hardIronOffset(3);
N = length(xc);

% prepare axes
largestRadius = -Inf;
smallestRadius = +Inf;
largestRadiusDirection = [NaN NaN NaN];
smallestRadiusDirection = [NaN NaN NaN];

% find largest and smallest axis
for i=1:N
    % Taking the square root is not really needed here because we only
    % need the direction of the longest or shortest axis
    radius = sqrt(xc(i).*xc(i) + yc(i).*yc(i) + zc(i).*zc(i));
        
    % Detect longest axis
    if radius > largestRadius
        largestRadius = radius;
        largestRadiusDirection = [xc(i) yc(i) zc(i)];
    end
    
    % Detect shortest axis
    if radius < smallestRadius
        smallestRadius = radius;
        smallestRadiusDirection = [xc(i) yc(i) zc(i)];
    end
end

% determine rotation angles from found vectors
[yaw, pitch, roll] = yawPitchRoll(largestRadiusDirection, smallestRadiusDirection);

% get rotation matrix

% add affine transformations to path
path(fullfile(fileparts(which(mfilename)), 'affine'), path)

% determine average radius to make it beautiful.
avgRadius = sqrt(mean(xc.*xc + yc.*yc + zc.*zc));

% get rotation matrix
Rx = affine_rotation_x(roll);
Ry = affine_rotation_y(pitch);
Rz = affine_rotation_z(yaw);
S  = affine_scale(2*avgRadius/smallestRadius, 2*avgRadius/largestRadius, 1);
A  = S*Rz*Ry*Rx;

iRx = affine_rotation_x(-roll);
iRy = affine_rotation_y(-pitch);
iRz = affine_rotation_z(-yaw);
iA  = iRx*iRy*iRz;

% prepare the test vector
test = [1 0 0];

% prepare the missing radius detection
smallestRadius = +Inf;
smallestRadiusDirection = [NaN NaN NaN];

% rotate all sampled vectors to align semi-axes with reference axes
test = A * [test'; 1];
for i=1:N
    v    = A * [xc(i); yc(i); zc(i); 1];
    
    xc(i) = v(1);
    yc(i) = v(2);
    zc(i) = v(3);
    
    % Detect the missing radius
    % Taking the square root is not really needed here because we only
    % need the direction of the longest or shortest axis
    radius = sqrt(xc(i).*xc(i) + yc(i).*yc(i) + zc(i).*zc(i));
    
    % Detect shortest axis
    if radius < smallestRadius
        smallestRadius = radius;
        smallestRadiusDirection = [xc(i) yc(i) zc(i)];
    end
end

S2  = affine_scale(0.5, 0.5, avgRadius/smallestRadius);

% rotate back all sampled vectors to re-align with original orientation
test = iA * S2 * test;
for i=1:N
    % scale missing axis and then transform back
    v    = iA * S2 * [xc(i); yc(i); zc(i); 1];
    xc(i) = v(1);
    yc(i) = v(2);
    zc(i) = v(3);
end

% Generate combined correction matrix
correction = iA * S2 * A;
correction(1,4) = -hardIronOffset(1);
correction(2,4) = -hardIronOffset(2);
correction(3,4) = -hardIronOffset(3);

disp(' ');
disp('Calculated and soft-iron correction matrix:');
disp(num2str(correction(1:3, 1:3)));

disp(' ');
disp('Affine hard- and soft-iron correction matrix:');
disp(num2str(correction));

% Apply combined correction matrix
for i=1:N
    v    = correction * [x(i); y(i); z(i); 1];
    xc(i) = v(1);
    yc(i) = v(2);
    zc(i) = v(3);
end

clear yaw pitch roll;

%% Plot data
figureHandle = figure('Name', 'Raw sensor data', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

% define base colors
lineColor(1, :) = [1 0.25 0]; % x axis
lineColor(2, :) = [0.5 1 0]; % y axis
lineColor(3, :) = [0 0.5 1]; % z axis
lineColor(4, :) = [1 1 1];
axesColor = [0.473 0.473 0.473];
plotBackground = [0.15 0.15 0.15];
titleColor = [1 1 1];

%% Compass
% compass: x/y axis
axisCompass(1) = subplot(3, 3, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', lineColor(1, :), ...
    'YGrid', 'on', ...
    'YColor', lineColor(2, :), ...
    'Color', plotBackground ...
    );

line(x, y, ...
    'Parent', axisCompass(1), ...
    'Color', lineColor(4, :) ...
    );
hold on;

axis square;
xlim([-1 1]);
ylim([-1 1]);

title('x/y plane', ...
    'Color', titleColor ...
    );
xlabel('B_x [Gs]');
ylabel('B_y [Gs]');

% compass: x/z axis
axisCompass(2) = subplot(3, 3, 4, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', lineColor(1, :), ...
    'YGrid', 'on', ...
    'YColor', lineColor(3, :), ...
    'Color', plotBackground ...
    );

line(x, z, ...
    'Parent', axisCompass(2), ...
    'Color', lineColor(4, :) ...
    );

axis square;
xlim([-1 1]);
ylim([-1 1]);

title('x/z plane', ...
    'Color', titleColor ...
    );
xlabel('B_x [Gs]');
ylabel('B_z [Gs]');

% compass: y/z axis
axisCompass(3) = subplot(3, 3, 7, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', lineColor(2, :), ...
    'YGrid', 'on', ...
    'YColor', lineColor(3, :), ...
    'Color', plotBackground ...
    );

line(y, z, ...
    'Parent', axisCompass(3), ...
    'Color', lineColor(4, :) ...
    );

axis square;
xlim([-1 1]);
ylim([-1 1]);

title('y/z plane', ...
    'Color', titleColor ...
    );
xlabel('B_y [Gs]');
ylabel('B_z [Gs]');

%% Compass
% compass: x, y and z axis
axisCompass(4) = subplot(3, 3, [2 3 5 6 8 9], ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', lineColor(1, :), ...
    'YGrid', 'on', ...
    'YColor', lineColor(2, :), ...
    'ZGrid', 'on', ...
    'ZColor', lineColor(3, :), ...
    'Color', plotBackground, ...
    'LineStyle', ':' ...
    );

lol = line(x, y, z, ...
    'Parent', axisCompass(4), ...
    'Color', lineColor(4, :)*0.5...
    );

view(3);
axis square;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);

hold on;
line(xc, ...
    yc, ...
    zc, ...
    'Parent', axisCompass(4), ...
    'Color', [0.9 0.5 0], ...
    'LineStyle', '-' ...
    );

line([hardIronOffset(1) hardIronOffset(1)+1], ...
    [hardIronOffset(2) hardIronOffset(2)], ...
    [hardIronOffset(3) hardIronOffset(3)], ...
    'Parent', axisCompass(4), ...
    'Color', [0.7 0.7 0.7], ...
    'LineStyle', ':' ...
    );

line([0 test(1)], ...
    [0 test(2)], ...
    [0 test(3)], ...
    'Parent', axisCompass(4), ...
    'Color', [0.9 0.9 0.9], ...
    'LineStyle', '-' ...
    );

title('HMC5883L Magnetometer', ...
    'Color', titleColor ...
    );
xlabel('B_x [Gs]');
ylabel('B_y [Gs]');
zlabel('B_z [Gs]');
