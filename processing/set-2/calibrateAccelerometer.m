function xyzCalibrated = calibrateAccelerometer(xyz)
    [accelerometerCalibrationData, ~, ~] = loadCalibrationData();

    correction = accelerometerCalibrationData.correctionMatrix;

    vector = [ ...
        xyz(:,1)'; ...
        xyz(:,2)'; ...
        xyz(:,3)';
        ones(1, size(xyz,1))];
    xyzCalibrated = correction * vector;
    
    xyzCalibrated = xyzCalibrated(1:3,:)';
end