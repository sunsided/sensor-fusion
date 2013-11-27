clc; clear all; home;

% define the data set folder
dataSetFolder = '../../data/set-1/tilt-sphere';

%% Load the data
[~, ~, magnetometer, temperature] = loadData(dataSetFolder);

% Fetch axes
x = magnetometer(:, 2);
y = magnetometer(:, 3);
z = magnetometer(:, 4);    

% include Yury Petrov's ellipsoid fit
dataSetFolder = fullfile(fileparts(which(mfilename)), 'ellipsoid_fit');
path(dataSetFolder, path);

% fit ellipse
[center, radii, evecs, v] = ellipsoid_fit([x y z])

% hard iron correction
x = x - center(1);
y = y - center(2);
z = z - center(3);

%% Plot data
figureHandle = figure('Name', 'Magnetometer Calibration / Ellipse Fitting', ...
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

axisCompass(1) = subplot(1, 1, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', lineColor(1, :), ...
    'YGrid', 'on', ...
    'YColor', lineColor(2, :), ...
    'ZGrid', 'on', ...
    'ZColor', lineColor(3, :), ...
    'Color', plotBackground ...
    );

title('Ellipse Fitting (Yury Petrov)', ...
    'Parent', axisCompass(1), ...
    'Color', titleColor ...
    );

% plot
line(x, y, z, ...
    'Parent', axisCompass(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'Color', [0.8 0.8 0.8], ...
    'MarkerSize', 1);
hold on;

axis square
xlim([-1 1]); 
ylim([-1 1]); 
zlim([-1 1]);
view(3);

% prepare semi-axis plots
semiaxis(3, :) = evecs(:,1) * radii(1);
semiaxis(2, :) = evecs(:,2) * radii(2);
semiaxis(1, :) = evecs(:,3) * radii(3);
semiaxisVectorEnd(3, :) = evecs(:,1);
semiaxisVectorEnd(2, :) = evecs(:,2);
semiaxisVectorEnd(1, :) = evecs(:,3);

% plot semi-axes
line( [0 semiaxis(1,1)], ...
      [0 semiaxis(1,2)], ...
      [0 semiaxis(1,3)], ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(1, :), ...
      'LineWidth', 2)
line( [0 semiaxis(2,1)], ...
      [0 semiaxis(2,2)], ...
      [0 semiaxis(2,3)], ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(2, :), ...
      'LineWidth', 2)
line( [0 semiaxis(3,1)], ...
      [0 semiaxis(3,2)], ...
      [0 semiaxis(3,3)], ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(3, :), ...
      'LineWidth', 2)

% plot semi-axis vectors
line( [semiaxis(1, 1) semiaxisVectorEnd(1,1)], ...
      [semiaxis(1, 2) semiaxisVectorEnd(1,2)], ...
      [semiaxis(1, 3) semiaxisVectorEnd(1,3)], ...
      'LineStyle', ':', ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(1, :), ...
      'LineWidth', 2)
line( [semiaxis(2, 1) semiaxisVectorEnd(2,1)], ...
      [semiaxis(2, 2) semiaxisVectorEnd(2,2)], ...
      [semiaxis(2, 3) semiaxisVectorEnd(2,3)], ...
      'LineStyle', ':', ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(2, :), ...
      'LineWidth', 2)
line( [semiaxis(3, 1) semiaxisVectorEnd(3,1)], ...
      [semiaxis(3, 2) semiaxisVectorEnd(3,2)], ...
      [semiaxis(3, 3) semiaxisVectorEnd(3,3)], ...
      'LineStyle', ':', ...
      'Parent', axisCompass(1), ...
      'Color', lineColor(3, :), ...
      'LineWidth', 2)