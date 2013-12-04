function txyzCalibrated = calibrateAccelerometer(txyz, mode)

    [accelerometerCalibrationData, ~] = loadCalibrationData();

    % ellipsoid fitting disabled by default, because it does not seem to work that well
    if ~exist('mode', 'var')
        mode = 1;
    end
    
    if mode == 0
        offset = accelerometerCalibrationData.offset;
        if size(txyz,2) == 4
            txyzCalibrated = [ ...
                txyz(:,2) - offset(1), ...
                txyz(:,3) - offset(2), ...
                txyz(:,4) - offset(3)];
            txyzCalibrated = [txyz(:,1) txyzCalibrated(:, 1:3)];
        else % assume XYZ values
            txyzCalibrated = [ ...
                txyz(:,1) - offset(1), ...
                txyz(:,2) - offset(2), ...
                txyz(:,3) - offset(3)];
        end
    else
        correction = accelerometerCalibrationData.correctionMatrix;
        if size(txyz,2) == 4
            txyzCalibrated = correction * [ ...
                txyz(:,2); ...
                txyz(:,3); ...
                txyz(:,4);
                1];
            txyzCalibrated = [txyz(:,1) txyzCalibrated(:, 1:3)'];
        else % assume XYZ values
            txyzCalibrated = correction * [ ...
                txyz(:,1); ...
                txyz(:,2); ...
                txyz(:,3);
                1];
            txyzCalibrated = txyzCalibrated(1:3)';
        end
    end

end