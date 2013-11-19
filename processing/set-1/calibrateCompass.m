function txyzCalibrated = calibrateCompass(txyz)

    [~, compassCalibrationData] = loadCalibrationData();
    correctionMatrix = compassCalibrationData.correctionMatrix;

    N = size(txyz(:,1));
    txyzCalibrated = zeros(size(txyz));
    for i=1:N
        calibrated = correctionMatrix * [txyz(i,2:4) 1]';
        txyzCalibrated(i, 1:4) = [txyz(i,1) calibrated(1:3)'];
    end

end