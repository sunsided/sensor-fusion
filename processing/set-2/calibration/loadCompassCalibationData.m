%% Magnetometer calibration data
function data = loadCompassCalibationData
    
    % Load data functions
    path(fullfile(fileparts(which(mfilename)), '..'), path);

    % define the data set folder
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'full-sphere');

    % Load the data
    [~, ~, magnetometer, ~] = loadData(dataSetFolder, false);

    % Fetch axes
    x = magnetometer.Data(:, 1);
    y = magnetometer.Data(:, 2);
    z = magnetometer.Data(:, 3);
    
    % Calibrate sensor
    correction = calibrateByEllipseFitting(x, y, z);

    disp(' ');
    disp('Affine hard- and soft-iron correction matrix:');
    disp(num2str(correction));
    
    % Load the data: x pointing forward
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward');
    [~, ~, Zup, ~] = loadData(dataSetFolder, false);
    
    % Load the data: x pointing up
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-x-up');
    [~, ~, Xup, ~] = loadData(dataSetFolder, false);
    
    % Load the data: z pointing left
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-y-down');
    [~, ~, Zfront, ~] = loadData(dataSetFolder, false);
    
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
    
    % Return data
    data = struct( ...
        'variance', variances, ...
        'correctionMatrix', correction);
    
end