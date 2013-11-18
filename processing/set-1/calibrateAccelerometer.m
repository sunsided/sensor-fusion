function txyzCalibrated = calibrateAccelerometer(txyz)

    [accelerometerCalibrationData, ~] = loadCalibrationData();
    offset = accelerometerCalibrationData.offset;

    txyzCalibrated = [ ...
        txyz(:,2) - offset(1), ...
        txyz(:,3) - offset(2), ...
        txyz(:,4) - offset(3)];
    txyzCalibrated = [txyz(:,1) txyzCalibrated(:, 1:3)];

end