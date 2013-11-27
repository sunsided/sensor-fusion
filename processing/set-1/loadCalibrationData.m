function [accelerometer, magnetometer] =  loadCalibrationData
% loadCalibrationData Loads sensor calibration data and populates global variables with the data to be used by the calibration functions.

    %% Load magnetometer data
    global compassCalibrationData
    if ~exist('compassCalibrationData', 'var') || isempty(compassCalibrationData)
        disp('Loading magnetometer calibration data.');
        compassCalibrationData = loadCompassCalibationData();
    else
        %disp('Using existing compass calibration data.');
    end
    magnetometer = compassCalibrationData;

    %% Load accelerometer data
    global accelerometerCalibrationData
    if ~exist('accelerometerCalibrationData', 'var') || isempty(accelerometerCalibrationData)
        disp('Loading accelerometer calibration data.');
        accelerometerCalibrationData = loadAccelerometerCalibrationData();
    else
        %disp('Using existing accelerometer calibration data.');
    end
    accelerometer = accelerometerCalibrationData;
    
end

%% Magnetometer calibration data
function data = loadCompassCalibationData
    
    % define the data set folder
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-1', 'tilt-sphere');
    path(dataSetFolder, path);

    % Load the data
    [~, ~, magnetometer, ~] = loadData(dataSetFolder);

    % Fetch axes
    x = magnetometer(:, 2);
    y = magnetometer(:, 3);
    z = magnetometer(:, 4);    
    
    % Calculate hard-iron factor
    % In theory, a simple average (mean(magnetometer(:, 2:4)))
    % would do the trick, but that does not take into account the
    % actual number of measurements taken. Averging the component-wise
    % min and max values yields better results but is prone to errors
    % due to outliers in the measurement.
    hardIronOffset(1) = (max(x)+min(x))/2;
    hardIronOffset(2) = (max(y)+min(y))/2;
    hardIronOffset(3) = (max(z)+min(z))/2;
    
    % TODO: Calculate soft-iron factors
    % A naive approach: measure all distances from the center point
    % to each point in the point cloud. The largest and smallest distances
    % are equal to the major and minor axis of the ellipse, respectively.
    % Take the angles to these vectors and rotate to X/Y, then scale to
    % unity. Repeat to find the third axis.
    
    disp('Calculated hard iron offset: ');
    disp(num2str(hardIronOffset));
    
    % Calculate soft-iron factors
    % A naive approach: measure all distances from the center point
    % to each point in the point cloud. The largest and smallest distances
    % are equal to the major and minor axis of the ellipse, respectively.
    % Take the angles to these vectors and rotate to X/Y, then scale to
    % unity. Repeat to find the third axis.
    xc = x - hardIronOffset(1);
    yc = y - hardIronOffset(2);
    zc = z - hardIronOffset(3);
    N = length(xc);

    % prepare axes
    largestRadiusSq = -Inf;
    smallestRadiusSq = +Inf;
    largestRadiusDirection = [NaN NaN NaN];
    smallestRadiusDirection = [NaN NaN NaN];

    % find largest and smallest axis
    for i=1:N
        % Taking the square root is not really needed here because we only
        % need the direction of the longest or shortest axis
        radiusSq = xc(i).*xc(i) + yc(i).*yc(i) + zc(i).*zc(i);

        % Detect longest axis
        if radiusSq > largestRadiusSq
            largestRadiusSq = radiusSq;
            largestRadiusDirection = [xc(i) yc(i) zc(i)];
        end

        % Detect shortest axis
        if radiusSq < smallestRadiusSq
            smallestRadiusSq = radiusSq;
            smallestRadiusDirection = [xc(i) yc(i) zc(i)];
        end
    end

    % draw roots
    smallestRadius = sqrt(smallestRadiusSq);
    largestRadius = sqrt(largestRadiusSq);
    
    % determine rotation angles from found vectors
    [yaw, pitch, roll] = yawPitchRoll(largestRadiusDirection, smallestRadiusDirection);

    % add affine transformations to path
    path(fullfile(fileparts(which(mfilename)), 'affine'), path)

    % determine average radius to make it beautiful.
    avgRadius = sqrt(mean(xc.*xc + yc.*yc + zc.*zc));

    % get rotation matrix
    Rx = affine_rotation_x(roll);
    Ry = affine_rotation_y(pitch);
    Rz = affine_rotation_z(yaw);
    S  = affine_scale(2*avgRadius/smallestRadius, 2*avgRadius/largestRadius, 1);
    A  = S*Rz*Ry*Rx;

    iRx = affine_rotation_x(-roll);
    iRy = affine_rotation_y(-pitch);
    iRz = affine_rotation_z(-yaw);
    iA  = iRx*iRy*iRz;

    % prepare the test vector
    test = [1 0 0];

    % prepare the missing radius detection
    clear smallestRadius smallestRadiusSq largestRadius largestRadiusSq
    smallestRadius = +Inf;
    %smallestRadiusDirection = [NaN NaN NaN];

    % rotate all sampled vectors to align semi-axes with reference axes
    test = A * [test'; 1];
    for i=1:N
        v    = A * [xc(i); yc(i); zc(i); 1];

        xc(i) = v(1);
        yc(i) = v(2);
        zc(i) = v(3);

        % Detect the missing radius
        % Taking the square root is not really needed here because we only
        % need the direction of the longest or shortest axis
        radiusSq = xc(i).*xc(i) + yc(i).*yc(i) + zc(i).*zc(i);

        % Detect shortest axis
        if radiusSq < smallestRadius
            smallestRadiusSq = radiusSq;
            
            % should really rotate again here, but 
            % we just assume it's the missing axis
            %smallestRadiusDirection = [xc(i) yc(i) zc(i)];
        end
    end

    smallestRadius = sqrt(smallestRadiusSq);
    S2  = affine_scale(0.5, 0.5, avgRadius/smallestRadius);

    % Generate combined correction matrix
    correction = iA * S2 * A;
    correction(1,4) = -hardIronOffset(1);
    correction(2,4) = -hardIronOffset(2);
    correction(3,4) = -hardIronOffset(3);

    disp(' ');
    disp('Calculated and soft-iron correction matrix:');
    disp(num2str(correction(1:3, 1:3)));

    disp(' ');
    disp('Affine hard- and soft-iron correction matrix:');
    disp(num2str(correction));
    
    % Return data
    data = struct('hardIronOffset', hardIronOffset, 'correctionMatrix', correction);
    
end

%% Accelerometer calibration data
function data = loadAccelerometerCalibrationData

    % Load the data: x pointing forward
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-1', 'unmoved-x-pointing-forward');
    path(dataSetFolder, path);
    [accelerometerZup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: x pointing up
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-1', 'unmoved-x-pointing-up');
    path(dataSetFolder, path);
    [accelerometerXup, ~, ~, ~] = loadData(dataSetFolder);
    
    % Load the data: z pointing left
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', 'data', 'set-1', 'unmoved-z-pointing-left');
    path(dataSetFolder, path);
    [accelerometerZleft, ~, ~, ~] = loadData(dataSetFolder);

    % Determine x-axis offset and variance
    x = [accelerometerZup(:,2); accelerometerZleft(:,2)];
    xMean       = mean(x);
    xVariance   = var(x);
    
    % Determine y-axis offset and variance
    y = [accelerometerZup(:,3); accelerometerXup(:,3)];
    yMean       = mean(y);
    yVariance   = var(y);
    
    % Determine z-axis offset and variance
    z = [accelerometerXup(:,4); accelerometerZleft(:,4)];
    zMean       = mean(z);
    zVariance   = var(z);
    
    % Prepare packages
    means = [xMean, yMean, zMean];
    variances = [xVariance, yVariance, zVariance];
    
    disp('Calculated axis offset: ');
    disp(num2str(means));
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));
    
    % Return data
    data = struct(...
        'offset', means, ...
        'variance', variances ...
        );
    
end