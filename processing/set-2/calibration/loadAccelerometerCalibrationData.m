%% Accelerometer calibration data
function data = loadAccelerometerCalibrationData

    % Load data functions
    path(fullfile(fileparts(which(mfilename)), '..'), path);

    %% Calibrate using ellipsoid fitting
    
     % define the data set folder
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'full-sphere');

    % Load the data
    [accelerometer, ~, ~, ~] = loadData(dataSetFolder, false);
    
    % Fetch axes
    x = accelerometer.Data(:, 1);
    y = accelerometer.Data(:, 2);
    z = accelerometer.Data(:, 3);
    
    % Calibrate sensor
    correction = calibrateByEllipseFitting(x, y, z);

    disp(' ');
    disp('Affine correction matrix from ellipsoid fitting:');
    disp(num2str(correction));
    

    % Load the data: x pointing forward
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward-10min');
    [Zup, ~, ~, ~] = loadData(dataSetFolder, false);
    
    % Load the data: x pointing up
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-x-up');
    [Xup, ~, ~, ~] = loadData(dataSetFolder, false);
    
    % Load the data: z pointing left
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-y-down');
    [Zfront, ~, ~, ~] = loadData(dataSetFolder, false);

    %{
    % Determine x-axis offset and variance
    x = [Zup.Data(:,1); Zleft.Data(:,1)];
    xMean       = mean(x);
    xVariance   = var(x);
    
    % Determine y-axis offset and variance
    y = [Zup.Data(:,2); Xup.Data(:,2)];
    yMean       = mean(y);
    yVariance   = var(y);
    
    % Determine z-axis offset and variance
    z = [Xup.Data(:,3); Zfront.Data(:,3)];
    zMean       = mean(z);
    zVariance   = var(z);
    
    % Prepare packages
    means = [xMean, yMean, zMean];
    variances = [xVariance, yVariance, zVariance];
    
    disp('Calculated axis offset: ');
    disp(num2str(means));
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));
    %}
    
    % perform drift correction
    [~,    Zup] = determineDrift(Zup);
    [~,    Xup] = determineDrift(Xup);
    [~, Zfront] = determineDrift(Zfront);
    
    % get X axis variances and drift
    variancesX = [var(Zup.Data(:,1));
                 var(Zfront.Data(:,1))];
    %factorsX = [pZup(1,:); pZfront(1,:)];
    
    % get Y axis variances and drift
    variancesY = [var(Zup.Data(:,2));
                var(Xup.Data(:,2))];
    %factorsY = [pZup(2,:); pXup(2,:)];

    % get Z axis variances and drift
    variancesZ = [var(Xup.Data(:,3));
                var(Zfront.Data(:,3))];
	%factorsZ = [pXup(3,:); pZfront(3,:)];

    % combine pooled variances
    xVariance = ((Zup.Length - 1) * variancesX(1) + (Zfront.Length - 1) * variancesX(2)) / ...
                (Zup.Length + Zfront.Length - 2);
    
    yVariance = ((Zup.Length - 1) * variancesY(1) + (Xup.Length - 1) * variancesY(2)) / ...
                (Zup.Length + Xup.Length - 2);
            
    zVariance = ((Xup.Length - 1) * variancesZ(1) + (Zfront.Length - 1) * variancesZ(2)) / ...
                (Xup.Length + Zfront.Length - 2);
            
    % Prepare packages
    variances = [xVariance, yVariance, zVariance];
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));

    %% Return values
    
    % Return data
    data = struct(...
        'variance', variances, ...
        'correctionMatrix', correction ...
        );
    
end