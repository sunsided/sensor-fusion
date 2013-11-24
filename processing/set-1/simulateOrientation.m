clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

%% Prepare Plots

% define base colors
lineColor(1, :) = [1 0.25 0]; % x axis
lineColor(2, :) = [0.5 1 0]; % y axis
lineColor(3, :) = [0 0.5 1]; % z axis
axesColor = [0.473 0.473 0.473];
plotBackground = [0.15 0.15 0.15];
titleColor = [1 1 1];

%% Prepare Coordinate System Plot
figureHandle(1) = figure('Name', 'Coordinate System', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

axisCoordinate(1) = subplot(1, 1, 1, ...
    'Parent', figureHandle(1), ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'ZGrid', 'on', ...
    'ZColor', axesColor, ...
    'Color', plotBackground ...
    );

coordSys(1) = line(NaN, NaN, NaN, ...
    'Color', lineColor(1,:), ...
    'LineWidth', 3); hold on;
coordSys(2) = line(NaN, NaN, NaN, ...
    'Color', lineColor(2,:), ...
    'LineWidth', 3);
coordSys(3) = line(NaN, NaN, NaN, ...
    'Color', lineColor(3,:), ...
    'LineWidth', 3);
coordSys(4) = line(NaN, NaN, NaN, ...
    'Color', lineColor(3,:), ...
    'LineWidth', 1);
coordSys(5) = line(NaN, NaN, NaN, ...
    'Color', lineColor(1,:), ...
    'LineWidth', 1);
view(3);
xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);
grid on;
axis equal;

camproj('perspective');
view(-90, 15);
rotate3d;

%% Prepare Rotation Plots
figureHandle(2) = figure('Name', 'Orientation', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

% 3D
orientation(1) = subplot(3, 3, [2 3 5 6 8 9], ...
    'Parent', figureHandle(2), ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

plotHandle(1) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
plotHandle(2) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
axis square; grid on;

camproj('perspective');
view(-90, 15);
rotate3d;

% front
orientation(2) = subplot(3, 3, 1, ...
    'Parent', figureHandle(2), ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

plotHandle(3) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
plotHandle(4) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

title('front view (X/Z)', 'Color', titleColor);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
axis square; grid on;

view(0, 0);

% top
orientation(3) = subplot(3, 3, 4, ...
    'Parent', figureHandle(2), ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

plotHandle(5) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
plotHandle(6) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

title('top view (X/Y)', 'Color', titleColor);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
axis square; grid on;

view(-90, 90);

% left
orientation(4) = subplot(3, 3, 7, ...
    'Parent', figureHandle(2), ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );
axis square;

% Prepare plot
plotHandle(7) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
plotHandle(8) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

title('side view (Y/Z)', 'Color', titleColor);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
axis square; grid on;

view(-90, 0);

%% Animation
N = min(size(accelerometer,1), size(magnetometer,1));
for n=1:N

    % Fetch accelerometer axes
    a = accelerometer(n, 2:4);
    
    % Fetch magnetometer axes
    m = [-magnetometer(n, 3);
         -magnetometer(n, 2);
          magnetometer(n, 4)]';
    
    % Calibrate values
    a = calibrateAccelerometer(a);
    m = calibrateCompass(m);
      
    % Normalize for later use
    an = a/norm(a);
    mn = m/norm(m);
          
    % Debugging
    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                    a(1), a(2), a(3), m(1), m(2), m(3));
    disp(msg);
    
    % Fetch rotation
    [~, ~, ~, DCM, coordinateSystem] = yawPitchRoll(a, m);
    
    % prepare rotation matrices
    R = DCM;

    % prepare vertices
    vertices = [ 0.75  0     0.02;
                 0.25  0.5   0.02;
                 0.25  0.25  0.02;
                -0.75  0.25  0.02;
                -0.75 -0.25  0.02;
                 0.25 -0.25  0.02;
                 0.25 -0.5   0.02;
                 0.75  0     0.02
                ];
    N = size(vertices,1);
    vertices = [vertices; vertices];
    for vi=N+1:2*N
        vertices(vi,:) = vertices(vi,:) + [0 0 -0.04];
    end

    % transform vertices
    for vi=1:size(vertices,1)
        vx = vertices(vi,:);
        v = R*vertices(vi,:)';
        vertices(vi,:) = v';
    end    
    
    % Set data and draw
    for p=0:3
        set(plotHandle(p*2+1), 'XData', vertices(1:8,1), 'YData', vertices(1:8,2), 'ZData', vertices(1:8,3));
        set(plotHandle(p*2+2), 'XData', vertices((N+1):2*N,1), 'YData', vertices((N+1):2*N,2), 'ZData', vertices((N+1):2*N,3));
    end
    
    % Set coordinate system
    for ca=1:3
        set(coordSys(ca), 'XData', [0 coordinateSystem(ca,1)], ...
            'YData', [0 coordinateSystem(ca,2)], ...
            'ZData', [0 coordinateSystem(ca,3)]);
    end
    
    % accelerometer axis
    set(coordSys(4), 'XData', [0 an(1)*2], ...
            'YData', [0 an(2)*2], ...
            'ZData', [0 an(3)*2]);
    
    % magnetometer
    set(coordSys(5), 'XData', [0 mn(1)*2], ...
        'YData', [0 mn(2)*2], ...
        'ZData', [0 mn(3)*2]);
    drawnow;
end