function [xyzCalibrated, correction] = calibrateGyroscope(xyz)

    [~, gyroscopeCalibrationData, ~] = loadCalibrationData();
    correction = gyroscopeCalibrationData.correctionMatrix;

    vector = [ ...
        xyz(:,1)'; ...
        xyz(:,2)'; ...
        xyz(:,3)';
        ones(1, size(xyz,1))];
    xyzCalibrated = correction * vector;
    
    xyzCalibrated = xyzCalibrated(1:3,:)';
    correction = gyroscopeCalibrationData;
end