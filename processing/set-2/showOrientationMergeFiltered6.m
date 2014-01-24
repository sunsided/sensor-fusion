clear all; home;

%% Load the data
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-forward');
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-y-pointing-left');
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

% Initial integrational drift values taken from uncorrected
% steady measurements.
ix = .017;
iy = .0035;
iz = .004;

x = [0, ... % yaw   angles
     0, ... %       angular velocities
     0, ... %       angular accelerations
    iz, ... %       gyro integrational drift
     0, ... % pitch angles
     0, ... %       angular velocities
     0, ... %       angular accelerations
    iy, ... %       gyro integrational drift
     0, ... % roll  angles
     0, ... %       angular velocities
     0, ... %       angular accelerations
    ix]';   %       gyro integrational drift
 
% Estimated cycle time
T = .1;

% see: "Position Recovery from Accelerometric Sensors"
%       Antonio Filieri, Rossella Melchiotti
% see: "Error Reduction Techniques for a MEMS Accelerometer-based Digital
%       Input Device" (Appendix B)
%       Tsang Chi Chiu

%qc = 0.166^2;
qc = 20^2;

qtt = (1/20*qc*T^5); % cov(theta, theta)
qto = (1/8*qc*T^4);  % cov(theta, omega)
qtb = (1/8*qc*T^4);  % cov(theta, omega)
qta = (1/6*qc*T^3);  % cov(theta, alpha)

qoo = (1/3*qc*T^3);  % var(omega)
qoa = (1/2*qc*T^2);  % cov(theta, alpha)

qaa = (qc*T);        % var(theta)

qbb = (1/3*qc*T^3);  % var(bias)

Q = [
    qtt qto qta qtb      0   0   0   0      0   0   0   0;
    qto qoo qoa   0      0   0   0   0      0   0   0   0;
    qta qoa qaa   0      0   0   0   0      0   0   0   0;
    qtb   0   0 qbb      0   0   0   0      0   0   0   0;

      0   0   0   0    qtt qto qta qtb      0   0   0   0;
      0   0   0   0    qto qoo qoa   0      0   0   0   0;
      0   0   0   0    qta qoa qaa   0      0   0   0   0;
      0   0   0   0    qtb   0   0 qbb      0   0   0   0;

      0   0   0   0      0   0   0   0    qtt qto qta qtb;
      0   0   0   0      0   0   0   0    qto qoo qoa   0;
      0   0   0   0      0   0   0   0    qta qoa qaa   0;
      0   0   0   0      0   0   0   0    qtb   0   0 qbb];

% state covariance matrix
gv = 2*gyroscope.UserData.variance;
gV = gv.^2;
P = [
     90   0   0   0      0   0   0   0      0   0   0   0;
      0 gv(1) 0   0      0   0   0   0      0   0   0   0;
      0   0  30   0      0   0   0   0      0   0   0   0;
      0   0   0  .1      0   0   0   0      0   0   0   0;

      0   0   0   0     10   0   0   0      0   0   0   0;
      0   0   0   0      0 gv(2) 0   0      0   0   0   0;
      0   0   0   0      0   0  30   0      0   0   0   0;
      0   0   0   0      0   0   0  .1      0   0   0   0;

      0   0   0   0      0   0   0   0     90   0   0   0;
      0   0   0   0      0   0   0   0      0 gv(3) 0   0;
      0   0   0   0      0   0   0   0      0   0  30   0;
      0   0   0   0      0   0   0   0      0   0   0  .1];
   
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
    
    %[yaw, pitch, roll, DCM, coordinateSystem, ~] = yawPitchRoll(a, m);
    
    [roll, pitch, roll_error] = rollPitchFromAccelerometer(a, 0.01);
    [yaw] = yawFromMagnetometer(m, roll, pitch);
    
    current_ypr = [yaw, pitch, roll];
    ypr(i,:) = current_ypr;
    
    % get time derivative
    T = 0.1;
    ypr_gyro_current = [0 0 0];
    if i > 1
        T = gyroscope.Time(i) - gyroscope.Time(i-1);
        ypr_gyro_current = [gyroscope.Data(i, 3) -gyroscope.Data(i, 2) -gyroscope.Data(i, 1)];
        ypr_gyro(i, :) = ypr_gyro(i-1, :) + ypr_gyro_current * T;
    end
    
    % as for IDDCM ... Euler rates are not body rates.
    
    % Prepare Kalman Filter
   
    % state matrix
    A = [
         1 T 0.5*T^2 -T,     0 0       0  0,      0 0       0  0;
         0 1       T  0,     0 0       0  0,      0 0       0  0;
         0 0       1  0,     0 0       0  0,      0 0       0  0;
         0 0       0  1,     0 0       0  0,      0 0       0  0;
         
         0 0       0  0,     1 T 0.5*T^2 -T,      0 0       0  0;
         0 0       0  0,     0 1       T  0,      0 0       0  0;
         0 0       0  0,     0 0       1  0,      0 0       0  0;
         0 0       0  0,     0 0       0  1,      0 0       0  0;
     
         0 0       0  0,     0 0       0  0,      1 T 0.5*T^2 -T;
         0 0       0  0,     0 0       0  0,      0 1       T  0;
         0 0       0  0,     0 0       0  0,      0 0       1  0;
         0 0       0  0,     0 0       0  0,      0 0       0  1];
    
    % Kalman Filter: Initial Prediction
    if i == 1       
        x(1) = yaw;       
        x(5) = pitch;
        x(9) = roll;
    end
    
    % Update process noise
    % Integrational errors propagate from acceleration to
    % velocity and from velocity to position.
    
    %qc = 0.166^2;
    %dT = T*2;
    dT = T;
    
    qtt = (1/20*qc*dT^5); % cov(theta, theta)
    qto = (1/8*qc*dT^4);  % cov(theta, omega)
    qtb = (1/8*qc*dT^4);  % cov(theta, omega)
    qta = (1/6*qc*dT^3);  % cov(theta, alpha)
    
    qoo = (1/3*qc*dT^3);  % var(omega)
    qoa = (1/2*qc*dT^2);  % cov(theta, alpha)
    
    qaa = (qc*dT);        % var(theta)
    
    qbb = (1/3*qc*dT^3);  % var(bias)
    
    Q = [
        qtt qto qta qtb      0   0   0   0      0   0   0   0;
        qto qoo qoa   0      0   0   0   0      0   0   0   0;
        qta qoa qaa   0      0   0   0   0      0   0   0   0;
        qtb   0   0 qbb      0   0   0   0      0   0   0   0;

          0   0   0   0    qtt qto qta qtb      0   0   0   0;
          0   0   0   0    qto qoo qoa   0      0   0   0   0;
          0   0   0   0    qta qoa qaa   0      0   0   0   0;
          0   0   0   0    qtb   0   0 qbb      0   0   0   0;

          0   0   0   0      0   0   0   0    qtt qto qta qtb;
          0   0   0   0      0   0   0   0    qto qoo qoa   0;
          0   0   0   0      0   0   0   0    qta qoa qaa   0;
          0   0   0   0      0   0   0   0    qtb   0   0 qbb];
      
    
    % Kalman Filter: Predict
    [x, P] = kf_predict(x, A, P, lambda, Q);
           
    % Apply constraints to the estimated state
    %{
    if yaw < -160 && x(1) > 160
        x(1) = x(1) - 360;
    elseif yaw > 160 && x(1) < -160
        x(1) = x(1) + 360;
    end
        
    if roll < -160 && x(9) > 160
        x(9) = x(9) - 360;
    elseif roll > 160 && x(9) < -160
        x(9) = x(9) + 360;
    end
    %}
    
    % Note that roll and yaw estimation is still very bad in this approach
    % when pitch angle goes up to +/- ~40°
    
    % Detect sign for gyro integration of roll and yaw.
    direction = 1;
    if a(3) > 0
        direction = -1;
    end
    
    % Pitch threshold to avoid yaw and roll measurements
    % when in singularity
    SwitchThreshold = 0.1;
    if roll_error > SwitchThreshold

        % measurement transformation matrix
        H = [
             0 0 0 0 1 0 0 0 0 0 0 0;
             0 1 0 0 0 0 0 0 0 0 0 0;
             0 0 0 0 0 1 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 0 1 0 0];

        % Set variance in angle measurements
        Rp = 5; % degree error
         
        % measurement noise matrix
        R = [
           Rp    0    0    0;
            0  gv(1)  0    0;
            0    0  gv(2)  0;
            0    0    0   gv(3)];

        % Fetch mixed angle
%        roll_yaw_sum = atan2d(-DCM(2,1), DCM(2,2));

        % Measurement vector
        z = [
             pitch;
             ypr_gyro_current(1) * direction;
             ypr_gyro_current(2);
             ypr_gyro_current(3) * direction];
        
        % The next block prevents measuring angles altogether
         
        %{
        % measurement transformation matrix
        H = [
             0 1 0 0 0 0 0 0 0 0 0 0;
             0 0 0 0 0 1 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 0 1 0 0];

        % measurement noise matrix
        R = [
            gv(1)   0   0;
             0   gv(2)  0;
             0   0   gv(3)];

        % Measurement vector
        z = [
             ypr_gyro_current(1);
             ypr_gyro_current(2);
             ypr_gyro_current(3)];
        %}
        
    else
        
        % measurement transformation matrix
        H = [
             1 0 0 0 0 0 0 0 0 0 0 0;
             0 0 0 0 1 0 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 1 0 0 0;
             0 1 0 0 0 0 0 0 0 0 0 0;
             0 0 0 0 0 1 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 0 1 0 0];

        % Set variance in angle measurements
        Ry = 35;   % degree error
        Rp =  5; % degree error
        Rr = 35; % degree error
        
        % measurement noise matrix
        R = [
           Ry   0   0   0   0   0;
            0  Rp   0   0   0   0;
            0   0  Rr   0   0   0;
            0   0   0   gv(1)   0   0;
            0   0   0   0   gv(2)  0;
            0   0   0   0   0   gv(3)];

        % Fix angles
        if a(3) > 0
            yaw = yaw + 180;
            roll = roll - 180;
            
            while yaw > 180
                yaw = yaw - 360;
            end
            
            while yaw < -180
                yaw = yaw + 360;
            end
            
            while roll > 180
                roll = roll - 360;
            end
            
            while roll < -180
                roll = roll + 360;
            end
        end
        
        % Measurement vector
        z = [
             yaw;
             pitch;
             roll;
             ypr_gyro_current(1) * direction;
             %ypr_gyro_current(1);
             ypr_gyro_current(2);
             ypr_gyro_current(3) * direction];
             %ypr_gyro_current(3)];
     
    end

    % Kalman Filter: Measurement Update
    [x, P] = kf_update(x, z, P, H, R);
    
    % store state
    ypr_kf(i, :)    = [x(1)  x(5)  x(9)];
    omega_kf(i, :)  = [x(2), x(6), x(10)];
     
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