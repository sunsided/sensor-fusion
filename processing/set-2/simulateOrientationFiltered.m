clear all; close all; clc; home;

% define the data set folder
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-forward';
%dataSetFolder = '../../data/set-1/unmoved-x-pointing-up';
%dataSetFolder = '../../data/set-1/tilt-around-x-pointing-forward';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-z-pointing-up';
%dataSetFolder = '../../data/set-1/rotate-360ccw-around-x-pointing-forward';

%% Load the data
%dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'rotate-ccw-around-x-pointing-up');
dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-2', 'roll-and-tilt-at-45-90');
[accelerometer, gyroscope, magnetometer, temperature] = loadData(dataSetFolder, true);

% resample the time series
[accelerometer, magnetometer, gyroscope] = lerpTimeSeries(accelerometer, magnetometer, gyroscope);

%% Prepare affine transformations
path(fullfile(fileparts(which(mfilename)), 'affine'), path);

%% Prepare Plots
preparePlotOrientation();

%% Prepare Kalman Filter

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

%% Animation
n_step = 1;

N = accelerometer.Length;
for n=1:n_step:N

    % fetch RPY from accelerometer and magnetometer
    a = accelerometer.Data(n, :);
    m = magnetometer.Data(n, :);
    g = [-gyroscope.Data(n, 1) -gyroscope.Data(n, 2) gyroscope.Data(n, 3)];
    
    %{
    a = a / norm(a);
    g = g / norm(g);
    m = m / norm(m);
    %}
    
    % get time derivative
    T = 0.1;
    ypr_gyro_current = [0 0 0];
    if n > n_step
        T = gyroscope.Time(n) - gyroscope.Time(n-n_step);
    end
            
    %% Estimate DCM; Correct roll and pitch angle
    
    % Initialize state
    if n == 1       
        % Elements for the state matrix
        C31 = -a(1);
        C32 = -a(2);
        C33 = -a(3);
        
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
    q_rp = .1;
    Q_rp = [0 0 0,  0 0 0;
         0 0 0,  0 0 0;
         0 0 0,  0 0 0;

         0 0 0,  q_rp 0 0;
         0 0 0,  0 q_rp 0;
         0 0 0,  0 0 q_rp];
    
     if n == 1
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
    xn = norm(x_rp(1:3));
    C31 = x_rp(1)/xn;
    C32 = x_rp(2)/xn;
    C33 = x_rp(3)/xn;
            
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
             degtorad(g(1));
             degtorad(g(2));
             degtorad(g(3))];

    % Kalman Filter: Measurement Update
    [x_rp, P_rp] = kf_update(x_rp, z_rp, P_rp, H_rp, R_rp);
                
    % Fetch estimated state matrix elements
    % and renormalize them
    xn = norm(x_rp(1:3));
    C31 = x_rp(1)/xn;
    C32 = x_rp(2)/xn;
    C33 = x_rp(3)/xn;

    % calculate roll and pitch angles  
    pitchY = -asind(C31);
    rollX  = atan2d(C32, C33);
    
    %% Estimate DCM; Correct yaw angle
    
    % tilt-compensate magnetometer and fetch estimated 
    % yaw angle sine and cosine.
    
    Xh = m(1)*cosd(pitchY) + m(2)*sind(pitchY)*sind(rollX) + m(3)*sind(pitchY)*cosd(rollX);
    Yh =                     m(2)*cosd(rollX)              - m(3)*sind(rollX);
    
    yaw_sin = Yh / sqrt(Xh^2 + Yh^2);
    yaw_cos = Xh / sqrt(Xh^2 + Yh^2);
    
     % Initialize state
    if n == 1       
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
    q_y = .1;
    Q_y = [0 0 0,  0 0 0;
           0 0 0,  0 0 0;
           0 0 0,  0 0 0;

           0 0 0,  q_y 0 0;
           0 0 0,  0 q_y 0;
           0 0 0,  0 0 q_y];
    
     if n == 1
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
    xn = norm(x_y(1:3));
    C21 = x_y(1)/xn;
    C22 = x_y(2)/xn;
    C23 = x_y(3)/xn;    

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
   
    rgx =  beta * gv(1);
    rgy =  beta * gv(2);
    rgz =  beta * gv(3);
           
    R_y =  [mu 0  0,   0  0  0;
            0 mu  0,   0  0  0;
            0  0 mu,   0  0  0;
          
            0  0  0,  rgx 0  0;
            0  0  0,   0 rgy 0;
            0  0  0,   0  0 rgz];
     
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
           degtorad(g(1));
           degtorad(g(2));
           degtorad(g(3))];  

    % Kalman Filter: Measurement Update
    [x_y, P_y] = kf_update(x_y, z_y, P_y, H_y, R_y);
            
    % Fetch estimated state matrix elements
    % and renormalize them
    xn = norm(x_y(1:3));
    C21 = x_y(1)/xn;
    C22 = x_y(2)/xn;
    C23 = x_y(3)/xn;
       
    % Recreate remaining DCM row
    C1 = cross([C21 C22 C23], [C31 C32 C33]);
    C11 = C1(1);
    C12 = C1(2);
    C13 = C1(3);
    
    % Calculate yaw angle
    yawZ = atan2d(-C21, C11);
    
    %% Build the DCM
    
    DCM = [C11 C12 C13;
           C21 C22 C23;
           C31 C32 C33]';
    
    %% plot the orientation
    plotOrientation(DCM, a, m);
end