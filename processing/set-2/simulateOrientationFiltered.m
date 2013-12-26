clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-z-pointing-up';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-x-pointing-forward';

%% Load the data
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder, true);

% resample the time series
[accelerometer, magnetometer, gyroscope] = lerpTimeSeries(accelerometer, magnetometer, gyroscope);

%% Prepare affine transformations
path(fullfile(fileparts(which(mfilename)), 'affine'), path);

%% Prepare Plots
preparePlotOrientation();

%% Prepare Kalman Filter

% State variables:
% phi, psi, theta from integrated difference DCM
% phi, psi, theta from integrated gyro
% omega{phi, psi, theta} from direct gyro
% alpha{phi, psi, theta} from direct gyro
% gyro bias{phi, psi, theta}

% state matrix
x = [0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0]';

% state covariance matrix
vg(1) = 1; % 100*compass.UserData.variance(1);
vg(2) = 1; % 100*compass.UserData.variance(2);
vg(3) = 1; % 100*compass.UserData.variance(3);

P = [1.9 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1.9 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1.9, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     1 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, vg(1) 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 vg(2) 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 vg(3), 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 1 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 1 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 1, 0 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 1 0 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 1 0;
     0 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 1];
 
% state matrix
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
  
% measurement transformation matrix
H = [1 0 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 1 0, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 1, 0 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 1 0 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 1 0, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 1, 0 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 1 0 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 1 0, 0 0 0, 0 0 0;
     0 0 0, 0 0 0, 0 0 1, 0 0 0, 0 0 0];

% measurement noise matrix
%R = eye(size(P)) * 0.001;

% values taken from cov of diffs of ypr2/ypr_gyro and ypr_kf due to lack of
% better reference
R = [10.087 0 0,        0.1   -0.67  1.24,      0 0 0;
     0 6.1297 0,        -.18 -.065 0.43,        0 0 0;
     0 0 8.8969,        -.26 -.05  -.02,        0 0 0;
     .1 -.18 -.26,      11.7462 0 0,            0 0 0;
     -0.67 -.065 -.05,  0 10.365 0,             0 0 0;
     1.24 .43 -.02,     0 0 39.9315,            0 0 0;
     0 0 0,             0 0 0,                  1 0 0;
     0 0 0,             0 0 0,                  0 1 0;
     0 0 0,             0 0 0,                  0 0 1];
     

% Lambda coefficient for artificial increase of covariance
lambda = 0.98;

%% Animation
baseDCM = [];
oldDCM = zeros(3);
ypr = zeros(1,3);
ypr_last = zeros(1,3);
ypr_gyro = zeros(1,3);
ypr_gyro_last = zeros(1,3);
n_step = 2;

N = accelerometer.Length;
for n=1:n_step:N

    % Fetch sensor axes
    a = accelerometer.Data(n, :);
    m = magnetometer.Data(n, :);
    g = gyroscope.Data(n, :);
      
    % Normalize for later use
    an = a/norm(a);
    mn = m/norm(m);
    
    % Debugging
    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                    a(1), a(2), a(3), m(1), m(2), m(3));
    disp(msg);
    
    
    % Fetch rotation
    [yaw, pitch, roll, DCM, ~] = yawPitchRoll(a, m);
    
    % fetch RPY from integrated gyro
    ypr_gyro = [yaw, pitch, roll];
    ypr_gyro_current = [0 0 0];
    dt = 0.01;
    if n > 1
        dt = gyroscope.Time(n) - gyroscope.Time(n-n_step);
        
        ypr_gyro_current = [g(3) -g(2) -g(1)];
        ypr_gyro = ypr_gyro_last + ypr_gyro_current * dt;
    end
    ypr_gyro_last = ypr_gyro;
    
    % calculate the difference of the rotation and hence the angular velocity
    difference = DCM'*oldDCM;   
    om_pitchY = -asind(difference(1, 3));
    om_rollX = atan2d(difference(2, 3), difference(3, 3));
    om_yawZ = atan2d(difference(1, 2), difference(1, 1));
    
    % integrate the angular velocity
    ypr = [yaw pitch roll];
    if n > 1
        ypr = ypr_last + [om_yawZ, om_pitchY, om_rollX];
    end
    ypr_last = ypr;
        
    % save current DCM for next iteration
    oldDCM = DCM;
    
    % Kalman Filter: State Matrix
    T = dt;
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
    
    % Kalman Filter: Prediction
    if n == 1
        x(1:3) = [yaw pitch roll];
        x(4:6) = [yaw pitch roll];
        [x, P] = kf_predict(x, A, P, lambda);
    end
    
    % Measurement vector
    z = [ypr(1) ypr(2) ypr(3), ypr_gyro(1) ypr_gyro(2) ypr_gyro(3), ypr_gyro_current(1) ypr_gyro_current(2) ypr_gyro_current(3)]';
    
    % Kalman Filter: Measurement Update
    [x, P] = kf_update(x, z, P, H, R);
    
    % Transform system based on Kalman Filtered data
    roll   =  degtorad(x(1));
    pitch  = -degtorad(x(2));
    yaw    = -degtorad(x(3));
    rotation = affine_rotation_x(roll) * affine_rotation_y(pitch) * affine_rotation_z(yaw);
    
    % Extract DCM
    DCM = rotation(1:3, 1:3);
    
    % Transform coordinate system
    % Interestingly, this happens to be the same as the DCM.
    coordinateSystem = DCM*eye(3);
    
    % Change DCM base frame for display of the arrow
    %{
    if isempty(baseDCM)
        baseDCM = DCM';
    else
        DCM = DCM*baseDCM;
    end
    %}
            
    % Kalman Filter: Prediction
    [x, P] = kf_predict(x, A, P, lambda);
    
    % plot the orientation
    plotOrientation(DCM, coordinateSystem, an, mn);
end