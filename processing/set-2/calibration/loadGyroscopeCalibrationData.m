%% Magnetometer calibration data
function data = loadGyroscopeCalibrationData
    
    % Load data functions
    path(fullfile(fileparts(which(mfilename)), '..'), path);

    % Load the data: x pointing forward
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-x-pointing-forward-10min');
    [~, Zup, ~, ~] = loadData(dataSetFolder, false);
    
    % Load the data: x pointing up
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-x-up');
    [~, Xup, ~, ~] = loadData(dataSetFolder, false);
    
    % Load the data: z pointing left
    dataSetFolder = fullfile(fileparts(which(mfilename)), '..' , '..', '..', 'data', 'set-2', 'unmoved-with-z-pointing-front-and-y-down');
    [~, Zfront, ~, ~] = loadData(dataSetFolder, false);
    
    % perform drift correction
    [pZup,    Zup] = determineDrift(Zup);
    [pXup,    Xup] = determineDrift(Xup);
    [pZfront, Zfront] = determineDrift(Zfront);
    
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

    % combine pooled variances (weighted averages)
    xVariance = ((Zup.Length-1)*variancesX(1) + (Zfront.Length-1)*variancesX(2)) / ...
                (Zup.Length + Zfront.Length - 2);
    
    yVariance = ((Zup.Length-1)*variancesY(1) + (Xup.Length-1)*variancesY(2)) / ...
                (Zup.Length + Xup.Length - 2);
            
    zVariance = ((Xup.Length-1)*variancesZ(1) + (Zfront.Length-1)*variancesZ(2)) / ...
                (Xup.Length + Zfront.Length - 2);
            
    % Prepare packages
    variances = [xVariance, yVariance, zVariance];
    
    % combine offsets as a weighted average
    xOffset = ((Zup.Length-1)*pZup(1,2) + (Xup.Length-1)*pXup(1,2) + (Zfront.Length-1)*pZfront(1,2)) / ...
                (Zup.Length + Xup.Length + Zfront.Length - 3);
    
    yOffset = ((Zup.Length-1)*pZup(2,2) + (Xup.Length-1)*pXup(2,2) + (Zfront.Length-1)*pZfront(2,2)) / ...
                (Zup.Length + Xup.Length + Zfront.Length - 3);
            
    zOffset = ((Zup.Length-1)*pZup(3,2) + (Xup.Length-1)*pXup(3,2) + (Zfront.Length-1)*pZfront(3,2)) / ...
                (Zup.Length + Xup.Length + Zfront.Length - 3);

    % Prepare packages
    offsets = [xOffset, yOffset, zOffset];
            
    % Build affine correction matrix
    correction = eye(4);
    correction(1:3, 4) = -offsets';
    
    disp(' ');
    disp('Affine correction matrix:');
    disp(num2str(correction));
    
    disp('Calculated axis variances: ');
    disp(num2str(variances));
    
    disp('Calculated axis offsets: ');
    disp(num2str(offsets));
    
    % Return data
    data = struct( ...
        'variance', variances, ...
        'offset', offsets, ...
        'correctionMatrix', correction ...
        );
    
end