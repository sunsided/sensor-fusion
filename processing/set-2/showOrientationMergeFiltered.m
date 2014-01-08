clear all; home;

%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-forward');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-y-pointing-left');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-z-pointing-up');
[accelerometer, gyroscope, compass, ~] = loadData(dataSetFolder, true);

% resample the time series
[acceleration, gyroscope, compass] = lerpTimeSeries(accelerometer, gyroscope, compass);

% extract time vector
time = acceleration.Time;
N = acceleration.Length;

%% Prepare the Kalman filter

% State variables:
% phi, psi, theta from raw DCM
% phi, psi, theta from integrated difference DCM
% phi, psi, theta from integrated gyro
% omega{phi, psi, theta} from direct gyro
% alpha{phi, psi, theta} from direct gyro
% gyro bias{phi, psi, theta}

% state matrix
x = [0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0]';

% state covariance matrix
vg(1) = 1; % 100*compass.UserData.variance(1);
vg(2) = 1; % 100*compass.UserData.variance(2);
vg(3) = 1; % 100*compass.UserData.variance(3);

P = [
     1.1 0 0, 1 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1.1 0, 0 1 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1.1, 0 0 1, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     1 0 0,1.9 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1 0, 0 1.9 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1, 0 0 1.9, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 1 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 1 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 1, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, vg(1) 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 vg(2) 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 vg(3), 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1];
P_mask = P;
P_mask(P ~= 0) = 1;
 
% state matrix
%{
T = 0.1;
A = [1 0 0, 0 0 0, T 0 0, 0.5*T^2 0 0, 0 0 0;
     0 1 0, 0 0 0, 0 T 0, 0 0.5*T^2 0, 0 0 0;
     0 0 1, 0 0 0, 0 0 T, 0 0 0.5*T^2, 0 0 0;
     0 0 0, 1 0 0, T 0 0, 0.5*T^2 0 0, 0 0 0;
     0 0 0, 0 1 0, 0 T 0, 0 0.5*T^2 0, 0 0 0;
     0 0 0, 0 0 1, 0 0 T, 0 0 0.5*T^2, 0 0 0;
     0 0 0, 0 0 0, 1 0 0, T 0 0, 1 0 0;
     0 0 0, 0 0 0, 0 1 0, 0 T 0, 0 1 0;
     0 0 0, 0 0 0, 0 0 1, 0 0 T, 0 0 1;
     0 0 0, 0 0 0, 0 0 0, 1 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 0
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1];
%}  

% measurement transformation matrix
%{
H = [1 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 1 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 1 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 1, 0 0 0, 0 0 0];
%}

% measurement noise matrix
%R = eye(size(P)) * 0.001;

% values taken from cov of diffs of ypr2/ypr_gyro and ypr_kf due to lack of
% better reference
R = [15 0 0,     2 0 0, 2 0 0, 0 0 0;
     0 15 0,    0 2 0, 0 2 0, 0 0 0;
     0 0 15,     0 0 2, 0 0 2, 0 0 0;
     2 0 0,      10.087 0 0,        0.1   0.67  1.24,      0 0 0;
     0 2 0,      0 6.1297 0,         .18  .065 0.43,        0 0 0;
     0 0 2,       0 0 8.8969,        .26 .05  .02,        0 0 0;
     2 0 0,      .1 .18 .26,      3.7462 0 0,            0 0 0;
     0 2 0, 	 0.67 .065 .05,  0 3.365 0,             0 0 0;
     0 0 2,      1.24 .43 .02,     0 0 3.9315,            0 0 0;
     0 0 0,       0 0 0,             0 0 0,                  1 0 0;
     0 0 0,     0 0 0,             0 0 0,                  0 1 0;
     0 0 0,     0 0 0,             0 0 0,                  0 0 1];
     

% Lambda coefficient for artificial increase of covariance
lambda = 0.99;

%% Get roll, pitch and yaw
hwb = waitbar(0, 'Calculating states ...');

ypr = zeros(N, 3);
ypr2 = zeros(N, 3);
ypr_kf = zeros(N, 3);
ypr_gyro = zeros(N, 3);
omega_kf = zeros(N, 3);

oldDCM = zeros(3);

maxx = -Inf;
maxP = -Inf;
maxA = -Inf;
maxz = -Inf;
maxH = -Inf;
maxR = -Inf;

minx = +Inf;
minP = +Inf;
minA = +Inf;
minz = +Inf;
minH = +Inf;
minR = +Inf;

for i=1:N
    % fetch RPY from accelerometer and magnetometer
    a = acceleration.Data(i, :);
    m = compass.Data(i, :);
    [yaw, pitch, roll, DCM, coordinateSystem, ~] = yawPitchRoll(a, m);
    ypr(i,:) = [yaw, pitch, roll];
    
    % fetch RPY from integrated gyro
    ypr_gyro(i, :) = [yaw, pitch, roll];
    ypr_gyro_current = [0 0 0];
    if i > 1
        ypr_gyro_current = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        dt = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + ypr_gyro_current * dt;
    end
        
    % calculate the difference of the rotation and hence the 
    % angular velocity
    difference = DCM'*oldDCM;   
    om_pitchY = -asind(difference(1, 3));
    om_rollX = atan2d(difference(2, 3), difference(3, 3));
    om_yawZ = atan2d(difference(1, 2), difference(1, 1));
    
    % integrate the angular velocity
    old_ypr2 = [yaw pitch roll]; % ypr(i, :);
    if i > 1
        old_ypr2 = ypr2(i-1, :);
    end
    ypr2(i, :) = old_ypr2 + [om_yawZ, om_pitchY, om_rollX];
        
    % save current DCM for next iteration
    diffDCM = difference;
    oldDCM = DCM;

    % Prepare Kalman Filter
    T = 0.1;
    if i > 1
        T = gyroscope.Time(i) - gyroscope.Time(i-1);
    end
    
     % state matrix
    A = [1 0 0, 0 0 0, 0 0 0, T 0 0, 0.5*T^2 0 0, 0 0 0;
         0 1 0, 0 0 0, 0 0 0, 0 T 0, 0 0.5*T^2 0, 0 0 0;
         0 0 1, 0 0 0, 0 0 0, 0 0 T, 0 0 0.5*T^2, 0 0 0;
         0 0 0, 1 0 0, 0 0 0, T 0 0, 0.5*T^2 0 0, 0 0 0;
         0 0 0, 0 1 0, 0 0 0, 0 T 0, 0 0.5*T^2 0, 0 0 0;
         0 0 0, 0 0 1, 0 0 0, 0 0 T, 0 0 0.5*T^2, 0 0 0;
         0 0 0, 0 0 0, 1 0 0, T 0 0, 0.5*T^2 0 0, 0 0 0;
         0 0 0, 0 0 0, 0 1 0, 0 T 0, 0 0.5*T^2 0, 0 0 0;
         0 0 0, 0 0 0, 0 0 1, 0 0 T, 0 0 0.5*T^2, 0 0 0;
         0 0 0, 0 0 0, 0 0 0, 1 0 0, T 0 0, T 0 0;
         0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 T 0, 0 T 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 T, 0 0 T;
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0, 0 0 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 0 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 0
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1];
    
    % Kalman Filter: Initial Prediction
    if i == 1
        x(1:3) = [yaw pitch roll];
        x(4:6) = [yaw pitch roll];
        x(7:9) = [yaw pitch roll];
        [x, P] = kf_predict(x, A, P, lambda);
    end
         
    % measurement transformation matrix
    H = [1 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         0 1 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         0 0 1, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         1 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         0 0 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         0 0 0, 0 0 1, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
         0 0 0, 0 0 0, 1 0 0, 0 0 0, 0 0 0, T 0 0;
         0 0 0, 0 0 0, 0 1 0, 0 0 0, 0 0 0, 0 T 0;
         0 0 0, 0 0 0, 0 0 1, 0 0 0, 0 0 0, 0 0 T;
         0 0 0, 0 0 0, 0 0 0, 1 0 0, 0 0 0, 1 0 0;
         0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 0 0, 0 1 0;
         0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 0, 0 0 1];
    
    % Measurement vector
    z = [ypr(i, 1) ypr(i, 2) ypr(i, 3), ...
         ypr2(i, 1) ypr2(i, 2) ypr2(i, 3), ...
         ypr_gyro(i, 1) ypr_gyro(i, 2) ypr_gyro(i, 3), ...
         ypr_gyro_current(1) ypr_gyro_current(2) ypr_gyro_current(3)]';
    
    maxx = max(max(x), maxx);
    maxA = max(max(max(A)), maxA);
    maxP = max(max(max(P)), maxP);
    maxz = max(max(z), maxz);
     
    minx = min(min(x), minx);
    minA = min(min(min(A)), minA);
    minP = min(min(min(P)), minP);
    minz = min(min(z), minz);
    
    % Select measurement noise for raw DCM angles 
    % according to pitch angle
    ap = abs(pitch);
    if ap > 45
        ape = (sind(abs(pitch))*sqrt(180))^2;
        R(1,1) = 10 + ape;
        R(2,2) = 10;
        R(3,3) = 10 + ape;
    else
        R(1,1) = 10;
        R(2,2) = 10;
        R(3,3) = 10;
    end
     
    % Kalman Filter: Measurement Update
    [x, P] = kf_update(x, z, P, H, R);
    
    maxx = max(max(x), maxx);
    maxP = max(max(max(P)), maxP);
    maxH = max(max(max(H)), maxH);
    maxR = max(max(max(R)), maxR);
    
    minx = min(min(x), minx);
    minP = min(min(min(P)), minP);
    minH = min(min(min(H)), minH);
    minR = min(min(min(R)), minR);
    
    % Cancel out covariances known to not exist
    %P = P .* P_mask;
    
    ypr_kf(i, :) = (x(1:3) + x(4:6) + x(7:9))./3;
    omega_kf(i, :) = [x(12), x(11), x(10)];
    
    % Kalman Filter: Predict
    [x, P] = kf_predict(x, A, P, lambda);
    
    waitbar(i/N, hwb);
end
close(hwb);

% clamp angles to -180..180
ypr2 = clampangle(ypr2);
ypr_gyro = clampangle(ypr_gyro);
ypr_kf = clampangle(ypr_kf);

% Stats
fprintf('\nSimple statistics\n');
fprintf('x range: %f .. %f\n', minx, maxx);
fprintf('z range: %f .. %f\n', minz, maxz);
fprintf('A range: %f .. %f\n', minA, maxA);
fprintf('P range: %f .. %f\n', minP, maxP);
fprintf('H range: %f .. %f\n', minH, maxH);
fprintf('R range: %f .. %f\n', minR, maxR);

%% Plot data
figureHandle = figure('Name', 'Raw and derived inertial sensor data', ...
    'NumberTitle', 'off', ...
    'Color', [0.027 0.211 0.259] ...
    );

% define base colors
lineColor(1, :) = [1 0.25 0]; % x axis
lineColor(2, :) = [0.5 1 0]; % y axis
lineColor(3, :) = [0 0.5 1]; % z axis

lineColor(4, :) = [1 1 0]; % x axis
lineColor(5, :) = [1 0 1]; % y axis
lineColor(6, :) = [1.0 0.75 0.25]; % z axis

meanColor = [1 1 1];
axesColor = [0.473 0.473 0.473];
plotBackground = [0.15 0.15 0.15];
titleColor = [1 1 1];


%% Roll
axisRpy(1) = subplot(3, 3, 1, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = ypr(:, 3);
line(t, roll, ...
    'Parent', axisRpy(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    ); 
roll = ypr2(:, 3);
line(t, roll, ...
    'Parent', axisRpy(1), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(2) = subplot(3, 3, 4, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = ypr(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(2), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
pitch = ypr2(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(2), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(3) = subplot(3, 3, 7, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = ypr(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
yaw = ypr2(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(3), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw', 'DCM int');
set(legendHandle, 'TextColor', [1 1 1]);


%% Roll
axisRpy(4) = subplot(3, 3, 2, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = ypr_gyro(:, 3);
line(t, roll, ...
    'Parent', axisRpy(4), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    );  
hold on;
roll = ypr_kf(:, 3);
line(t, roll, ...
    'Parent', axisRpy(4), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(5) = subplot(3, 3, 5, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = ypr_gyro(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(5), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
hold on;
pitch = ypr_kf(:, 2);
line(t, pitch, ...
    'Parent', axisRpy(5), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(6) = subplot(3, 3, 8, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = ypr_gyro(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(6), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
hold on;
yaw = ypr_kf(:, 1);
line(t, yaw, ...
    'Parent', axisRpy(6), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    );

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro int', 'kf');
set(legendHandle, 'TextColor', [1 1 1]);


%% Roll
axisRpy(7) = subplot(3, 3, 3, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
roll = -gyroscope.Data(:,1);
line(t, roll, ...
    'Parent', axisRpy(7), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    ); 
hold on;
roll = omega_kf(:,1);
line(t, roll, ...
    'Parent', axisRpy(7), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);

%% Pitch
axisRpy(8) = subplot(3, 3, 6, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
pitch = -gyroscope.Data(:,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
hold on;
pitch = omega_kf(:,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);

%% Yaw
axisRpy(9) = subplot(3, 3, 9, ...
    'Parent', figureHandle, ...
    'XGrid', 'on', ...
    'XColor', axesColor, ...
    'YGrid', 'on', ...
    'YColor', axesColor, ...
    'Color', plotBackground ...
    );

t = time;
yaw = gyroscope.Data(:,3);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
hold on;
yaw = omega_kf(:,3);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', [1 1 1] ...
    ); 

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('gyro', 'gyro_{kf}');
set(legendHandle, 'TextColor', [1 1 1]);



%% Plot refining
% link the axes
linkaxes(axisRpy, 'xy');