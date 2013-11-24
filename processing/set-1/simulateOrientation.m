clear all; clc; home;

% define the data set folder
dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';

%% Load the data
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder);

%% Plot data
figureHandle = figure('Name', 'Orientation', ...
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

orientation(1) = subplot(1, 1, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );
axis square;

% Prepare plot
plotHandle(1) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
plotHandle(2) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
xlabel('x'); ylabel('y'); zlabel('z');
grid on;

camproj('perspective');
view(-90, 40);
rotate3d;

%% Animation
N = min(size(accelerometer,1), size(magnetometer,1));
for n=1:N

    % Fetch accelerometer axes
    a = accelerometer(n, 2:4);
    
    % Fetch magnetometer axes
    m = [-magnetometer(n, 3);
         -magnetometer(n, 2);
          magnetometer(n, 4)];
    
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
    set(plotHandle(1), 'XData', vertices(1:8,1), 'YData', vertices(1:8,2), 'ZData', vertices(1:8,3));
    set(plotHandle(2), 'XData', vertices((N+1):2*N,1), 'YData', vertices((N+1):2*N,2), 'ZData', vertices((N+1):2*N,3));
    drawnow;
end