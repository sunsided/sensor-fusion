function txyzCalibrated = calibrateCompass(txyz)

    [~, compassCalibrationData] = loadCalibrationData();
    correctionMatrix = compassCalibrationData.correctionMatrix;

    N = size(txyz(:,1));
    
    if size(txyz, 2) == 4
        txyzCalibrated = zeros(size(txyz));
        for i=1:N
            calibrated = correctionMatrix * [txyz(i,2:4) 1]';
            txyzCalibrated(i, 1:4) = [txyz(i,1) calibrated(1:3)'];
        end
    else % assume XYZ
        txyzCalibrated = zeros(size(txyz));
        for i=1:N
            calibrated = correctionMatrix * [txyz(i,:) 1]';
            txyzCalibrated(i, :) = calibrated(1:3)';
        end
    end

end