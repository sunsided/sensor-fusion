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
[accelerometer, gyroscope, magnetometer, ~] = loadData(dataSetFolder, true);

% resample the time series
[acceleration, gyroscope, magnetometer] = lerpTimeSeries(accelerometer, gyroscope, magnetometer);

% extract time vector
time = acceleration.Time;
N = acceleration.Length;

N = min(N, 10000);
time = time(1:N);

%% Prepare the Kalman filter

% state vector
x_rp = [0, ...  % C(3,1) of DCM
     0, ...     % C(3,2) of DCM
     0, ...     % C(3,3) of DCM
     0, ...     % omega_x
     0, ...     % omega_y
     0]';       % omega_z

x_y = [0, ...   % C(2,1) of DCM
     0, ...     % C(2,2) of DCM
     0, ...     % C(2,3) of DCM
     0, ...     % mag_x
     0, ...     % mag_y
     0]';       % mag_z
 
% Fetch variances
gv = gyroscope.UserData.variance;
av = accelerometer.UserData.variance;
mv = magnetometer.UserData.variance;
   
% Lambda coefficient for artificial increase of uncertainty
lambda_rp = 1;
lambda_y  = 1;

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
    m = magnetometer.Data(i, :);
    g = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        
    % get time derivative
    T = 0.1;
    ypr_gyro_current = [0 0 0];
    if i > 1
        T = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + g * T;
    end       
    
    %% Estimate DCM; Correct roll and pitch angle
    
    % Initialize state
    if i == 1       
        % Elements for the state matrix
        C31 =  a(1);
        C32 =  a(2);
        C33 =  a(3);
        
        x_rp(1) = C31;
        x_rp(2) = C32;
        x_rp(3) = C33;
        
        x_rp(4) = g(1);
        x_rp(5) = g(2);
        x_rp(6) = g(3);
    end
        
    % State matrix
    A_rp = [0 0 0,     0 -C33  C32;
         0 0 0,   C33    0 -C31;
         0 0 0,  -C32  C31    0;
         
         0 0 0,     0 0 0;
         0 0 0,     0 0 0;
         0 0 0,     0 0 0];
    
    % Process noise
    q_rp = .001;
    Q_rp = [0 0 0,  0 0 0;
         0 0 0,  0 0 0;
         0 0 0,  0 0 0;

         0 0 0,  q_rp 0 0;
         0 0 0,  0 q_rp 0;
         0 0 0,  0 0 q_rp];
    
     if i == 1
         P_rp = [5 0 0,  0 0 0;
              0 5 0,  0 0 0;
              0 0 5,  0 0 0;

              0 0 0,  1 0 0;
              0 0 0,  0 1 0;
              0 0 0,  0 0 1];
     end
     
    % Kalman Filter: Predict
    [dx_rp, dP_rp] = kf_predict(x_rp, A_rp, P_rp, lambda_rp, Q_rp);

    % integrate state
    x_rp = x_rp + dx_rp*T;
    P_rp = P_rp + dP_rp*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x_rp(1:3));
    C31 = x_rp(1)/n;
    C32 = x_rp(2)/n;
    C33 = x_rp(3)/n;    
            
    % Rebuild state matrix with normalized elements
    A_rp = [0 0 0,     0 -C33  C32;
         0 0 0,   C33    0 -C31;
         0 0 0,  -C32  C31    0;
         
         0 0 0,     0 0 0;
         0 0 0,     0 0 0;
         0 0 0,     0 0 0];  
    
    % Measurement noise
    alpha = 1000;
    beta  = 1;
    
    rax = alpha * av(1);
    ray = alpha * av(2);
    raz = alpha * av(3);
    
    rgx =  beta * gv(1);
    rgy =  beta * gv(2);
    rgz =  beta * gv(3);
    
    R_rp = [rax 0  0,   0  0  0;
          0 ray 0,   0  0  0;
          0 0 raz,  0  0  0;
          
          0  0  0,  rgx 0  0;
          0  0  0,   0 rgy 0;
          0  0  0,   0  0 rgz];
     
    % Measurement matrix
    H_rp = [1 0 0,  0 0 0;
         0 1 0,  0 0 0;
         0 0 1,  0 0 0;
        
         0 0 0,  1 0 0;
         0 0 0,  0 1 0;
         0 0 0,  0 0 1];
      
    % Measurement vector
    z_rp = [-a(1);
         -a(2);
         -a(3);
          g(1);
          g(2);
          g(3)];    

    % Kalman Filter: Measurement Update
    [x_rp, dP_rp] = kf_update(x_rp, z_rp, P_rp, H_rp, R_rp);
            
    % integrate state
    P_rp = P_rp + dP_rp*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x_rp(1:3));
    C31 = x_rp(1)/n;
    C32 = x_rp(2)/n;
    C33 = x_rp(3)/n;

    % calculate roll and pitch angles  
    pitchY = -asind(C31);
    rollX  = atan2d(C32, C33);
    
    %% Estimate DCM; Correct yaw angle
    
    % tilt-compensate magnetometer and fetch estimated 
    % yaw angle sine and cosine.
    
    Xh_y = m(1)*cosd(pitchY) + m(2)*sind(pitchY)*sind(rollX) + m(3)*sind(pitchY)*cosd(rollX);
    Yh_y = m(2)*cosd(rollX) - m(3)*sind(rollX);
    
    yaw_sin = Yh_y / sqrt(Xh_y^2 + Yh_y^2);
    yaw_cos = Xh_y / sqrt(Xh_y^2 + Yh_y^2);
    
     % Initialize state
    if i == 1       
        C21 =  m(1);
        C22 =  m(2);
        C23 =  m(3);
        
        x_y(1) = C21;
        x_y(2) = C22;
        x_y(3) = C23;
        
        x_y(4) = m(1);
        x_y(5) = m(2);
        x_y(6) = m(3);
    end
    
    % State matrix
    A_y = [0 0 0,   0   -C23  C22;
           0 0 0,   C23    0 -C21;
           0 0 0,  -C22  C21    0;
         
           0 0 0,     0 0 0;
           0 0 0,     0 0 0;
           0 0 0,     0 0 0];
    
    % Process noise
    q_y = .001;
    Q_y = [0 0 0,  0 0 0;
           0 0 0,  0 0 0;
           0 0 0,  0 0 0;

           0 0 0,  q_y 0 0;
           0 0 0,  0 q_y 0;
           0 0 0,  0 0 q_y];
    
     if i == 1
         P_y = [5 0 0,  0 0 0;
              0 5 0,  0 0 0;
              0 0 5,  0 0 0;

              0 0 0,  1 0 0;
              0 0 0,  0 1 0;
              0 0 0,  0 0 1];
     end
     
    % Kalman Filter: Predict
    [dx_y, dP_y] = kf_predict(x_y, A_y, P_y, lambda_y, Q_y);

    % integrate state
    x_y = x_y + dx_y*T;
    P_y = P_y + dP_y*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x_y(1:3));
    C21 = x_y(1)/n;
    C22 = x_y(2)/n;
    C23 = x_y(3)/n;    
            
    % Rebuild state matrix with normalized elements
    A_y = [0 0 0,   0   -C23  C22;
           0 0 0,   C23    0 -C21;
           0 0 0,  -C22  C21    0;
         
           0 0 0,     0 0 0;
           0 0 0,     0 0 0;
           0 0 0,     0 0 0];
    
    % Measurement noise
    beta  = 1;
    
    mu = 0.01;
   
    rmx =  beta * mv(1);
    rmy =  beta * mv(2);
    rmz =  beta * mv(3);
    
    R_y =  [mu 0  0,   0  0  0;
            0 mu  0,   0  0  0;
            0  0 mu,   0  0  0;
          
            0  0  0,  rmx 0  0;
            0  0  0,   0 rmy 0;
            0  0  0,   0  0 rmz];
     
    % Measurement matrix
    H_y = [1 0 0,  0 0 0;
           0 1 0,  0 0 0;
           0 0 1,  0 0 0;
        
           0 0 0,  1 0 0;
           0 0 0,  0 1 0;
           0 0 0,  0 0 1];
      
    % Measurement vector
    z_y = [cosd(pitchY)*yaw_sin;
           cosd(rollX)*yaw_cos + sind(rollX)*sind(pitchY)*yaw_sin;
          -sind(rollX)*yaw_cos + cosd(rollX)*sind(pitchY)*yaw_sin;
           m(1);
           m(2);
           m(3)];    

    % Kalman Filter: Measurement Update
    [x_y, dP_y] = kf_update(x_y, z_y, P_y, H_y, R_y);
            
    % integrate state
    P_y = P_y + dP_y*T;
    
    % Fetch estimated state matrix elements
    % and renormalize them
    n = norm(x_y(1:3));
    C21 = x_y(1)/n;
    C22 = x_y(2)/n;
    C23 = x_y(3)/n;
       
    % Recreate remaining DCM row
    C1 = cross([C21 C22 C23], [C31 C32 C33]);
    C11 = C1(1);
    C12 = C1(2);
    C13 = C1(3);
    
    % Calculate yaw angle
    yawZ = atan2d(-C21, C11);
    
    %% Extract angles for fun and glory   
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