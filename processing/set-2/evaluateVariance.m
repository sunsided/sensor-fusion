clear all; clc;

% Define number of iterations
N = 200;

% Define number of measurements
M = 1000;

% Define sensor axis variances
variances_x = [2.0347e-06, ...
               1.9233e-06, ...
               2.3021e-06];

variances_z = [1.0022e-05, ...
               9.7704e-06, ...
               2.4444e-05];           
           
% Buffer for the maximum covariance
max_covariance = NaN(3,3);
    
for i_iteration = 1:N

    disp(['Iteration: ' num2str(i_iteration)]);
    
    % Define actual rotation in degree.
    roll_actual     = -179 + 358 * rand(1);
    pitch_actual    = -88  + 176 * rand(1);
    yaw_actual      = -179 + 358 * rand(1);

    % Build affine transformation matrix
    path(fullfile(fileparts(which(mfilename)), 'affine'), path);
    A = affine_rotation_z(degtorad(yaw_actual)) * ...
        affine_rotation_y(degtorad(pitch_actual)) * ...
        affine_rotation_x(degtorad(roll_actual));
    R = A(1:3, 1:3);

    % Define base axes
    % Y will be calculated on-the-fly using vector cross product
    X = R * [1 0 0]';
    Z = R * [0 0 1]';

    % Preallocate measurement buffer
    angles = NaN(M, 3);

    % Create noisy measurements
    for i_measurement = 1:M
        noise_x = sqrt(variances_x) .* randn(1,3);
        noise_z = sqrt(variances_z) .* randn(1,3);

        x = X' + noise_x;
        z = Z' + noise_z;

        % Normalize to get axes
        x = x/norm(x);
        z = z/norm(z);

        % Calcuate Y axis
        y = cross(z,x);
        y = y / norm(y);

        % Build direction cosine matrix
        DCM = [x; y; z];

        % Extract angles from DCM
        % see: William Premerlani, "Computing Euler Angles from Direction Cosines"
        pitchY = -asind(DCM(1, 3));
        rollX  =  atan2d(DCM(2, 3), DCM(3, 3));
        yawZ   =  atan2d(DCM(1, 2), DCM(1, 1));   

        % Save measurements
        angles(i_measurement, :) = [rollX, pitchY, yawZ];
    end

    % Remove mean (actual orientation) from the measured angles
    angles = [angles(:,1) - roll_actual, ...
              angles(:,2) - pitch_actual, ...
              angles(:,3) - yaw_actual];

    % Extract results
    %mean_error = mean(angles);
    covariance = cov(angles);

    max_covariance = max(max_covariance, covariance);
end