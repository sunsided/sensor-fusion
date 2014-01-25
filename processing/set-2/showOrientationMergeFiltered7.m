% This approach estimates the direction cosine matrix directly
% within the Kalman filter.
%
% See: "A DCM Based Orientation Estimation Algorithm with an
%       Inertial Measurement Unit and a Magnetic Compass"
%       Nguyen Ho Quoc Phuong, Hee-Jun Kang, Young-Soo Suh, Young-Sik Ro

clear all; home;

%% Load the data
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
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

N = min(N, 10000);
time = time(1:N);

%% Prepare the Kalman filter

% state vector
x = [0, ...     % C(3,1) of DCM
     0, ...     % C(3,2) of DCM
     0, ...     % C(3,3) of DCM
     0, ...     % omega_x
     0, ...     % omega_y
     0]';       % omega_z

% Fetch variances
gv = gyroscope.UserData.variance;
av = accelerometer.UserData.variance;
  
% state covariance matrix
P = [];
B = eye(size(P));
   
% Lambda coefficient for artificial increase of covariance
lambda = 1;

%% Get roll, pitch and yaw
hwb = waitbar(0, 'Calculating states ...');

ypr = zeros(N, 3);
ypr2 = zeros(N, 3);
ypr_kf = zeros(N, 3);
ypr_gyro = zeros(N, 3);
omega_kf = zeros(N, 3);

oldDCM = zeros(3);
ypr_base = [0 0 0];

for i=1:N
    % fetch RPY from accelerometer and magnetometer
    a = acceleration.Data(i, :);
    m = compass.Data(i, :);
    g = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        
    % fetch DCM from readings
    [~, ~, ~, DCM] = yawPitchRoll(a, m);
    
    % Turn around
    DCM = DCM';
    
    % get time derivative
    T = 0.1;
    ypr_gyro_current = [0 0 0];
    if i > 1
        T = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + g * T;
    end       
    
    % Initialize state
    if i == 1       
        % Elements for the state matrix
        C31 =  a(1);
        C32 =  a(2);
        C33 =  a(3);
        
        x(1) = C31;
        x(2) = C32;
        x(3) = C33;
        
        x(4) = g(1);
        x(5) = g(2);
        x(6) = g(3);
    end
        
    % State matrix
    A = [0 0 0,     0 -C33  C32;
         0 0 0,   C33    0 -C31;
         0 0 0,  -C32  C31    0;
         
         0 0 0,     0 0 0;
         0 0 0,     0 0 0;
         0 0 0,     0 0 0];
    
    % Process noise
    q = 1;
    Q = [0 0 0,  0 0 0;
         0 0 0,  0 0 0;
         0 0 0,  0 0 0;

         0 0 0,  q 0 0;
         0 0 0,  0 q 0;
         0 0 0,  0 0 q];
    
     if i == 1
         P = [5 0 0,  0 0 0;
              0 5 0,  0 0 0;
              0 0 1,  0 0 0;

              0 0 0,  1 0 0;
              0 0 0,  0 1 0;
              0 0 0,  0 0 1];
     end
     
    % Kalman Filter: Predict
    [dx, dP] = kf_predict(x, A, P, lambda, Q);

    % integrate state
    x = x + dx*T;
    P = P + dP*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x(1:3));
    C31 = x(1)/n;
    C32 = x(2)/n;
    C33 = x(3)/n;    
            
    % Rebuild state matrix with normalized elements
    A = [0 0 0,     0 -C33  C32;
         0 0 0,   C33    0 -C31;
         0 0 0,  -C32  C31    0;
         
         0 0 0,     0 0 0;
         0 0 0,     0 0 0;
         0 0 0,     0 0 0];  
    
    % Measurement noise
    alpha = 0.0001;
    beta  = 0.1;
    
    rax = 1/alpha * av(1);
    ray = 1/alpha * av(2);
    raz = 1/alpha * av(3);
    
    rgx =  beta * gv(1);
    rgy =  beta * gv(2);
    rgz =  beta * gv(3);
    
    R = [rax 0  0,   0  0  0;
          0 ray 0,   0  0  0;
          0  0 raz,  0  0  0;
          
          0  0  0,  rgx 0  0;
          0  0  0,   0 rgy 0;
          0  0  0,   0  0 rgz];
     
    % Measurement matrix
    H = [1 0 0,  0 0 0;
         0 1 0,  0 0 0;
         0 0 1,  0 0 0;
        
         0 0 0,  1 0 0;
         0 0 0,  0 1 0;
         0 0 0,  0 0 1];
      
    % Measurement vector
    z = [-a(1);
         -a(2);
         -a(3);
          g(1);
          g(2);
          g(3)];    

    % Kalman Filter: Measurement Update
    [x, dP] = kf_update(x, z, P, H, R);
            
    % integrate state
    P = P + dP*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x(1:3));
    C31 = x(1)/n;
    C32 = x(2)/n;
    C33 = x(3)/n;    

    % calculate roll and pitch angles  
    pitchY = -asind(C31);
    rollX  = atan2d(C32, C33);
    %yawZ   = atan2d(DCM(1, 2), DCM(1, 1));
    yawZ = NaN;
    
    ypr(i,:) = [yawZ, pitchY, rollX];
    
    % store state
    ypr_kf(i, :)    = [NaN, NaN, NaN];
    omega_kf(i, :)  = [NaN, NaN, NaN];
     
    waitbar(i/N, hwb);
end
close(hwb);

% clamp angles to -180..180
ypr2 = clampangle(ypr2);
ypr_gyro = clampangle(ypr_gyro);
ypr_kf = clampangle(ypr_kf);


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

xlim([0 t(end)]);
ylim([-180 180]);

title('Roll', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
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

xlim([0 t(end)]);
ylim([-180 180]);

title('Pitch (elevation)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
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

xlim([0 t(end)]);
ylim([-180 180]);

title('Yaw (azimuth, heading)', ...
    'Color', titleColor ...
    );
ylabel('angle [\circ]');
xlabel('t [s]');
legendHandle = legend('DCM raw');
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
roll = -gyroscope.Data(1:N,1);
line(t, roll, ...
    'Parent', axisRpy(7), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(4, :) ...
    ); 
hold on;
roll = omega_kf(1:N,3);
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
pitch = -gyroscope.Data(1:N,2);
line(t, pitch, ...
    'Parent', axisRpy(8), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(5, :) ...
    ); 
hold on;
pitch = omega_kf(1:N,2);
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
yaw = gyroscope.Data(1:N,3);
line(t, yaw, ...
    'Parent', axisRpy(9), ...
    'LineStyle', 'none', ...
    'Marker', '.', ...
    'MarkerSize', 2, ...
    'Color', lineColor(6, :) ...
    ); 
hold on;
yaw = omega_kf(1:N,1);
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