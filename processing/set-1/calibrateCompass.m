function txyzCalibrated = calibrateCompass(txyz)

    [~, compassCalibrationData] = loadCalibrationData();
    hardIronOffset = compassCalibrationData.hardIronOffset;

    txyzCalibrated = [ ...
        txyz(:,2) - hardIronOffset(1), ...
        txyz(:,3) - hardIronOffset(2), ...
        txyz(:,4) - hardIronOffset(3)];
    txyzCalibrated = [txyz(:,1) txyzCalibrated(:, 1:3)];

end