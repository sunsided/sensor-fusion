function txyzCalibrated = calibrateCompass(txyz)

    % Fetch axes
    x = txyz(:, 2);
    y = txyz(:, 3);
    z = txyz(:, 4);

    % Calculate hard-iron factor
    % In theory, a simple average (mean(magnetometer(:, 2:4)))
    % would do the trick, but that does not take into account the
    % actual number of measurements taken. Averging the component-wise
    % min and max values yields better results but is prone to errors
    % due to outliers in the measurement.
    hardIronOffset(1) = (max(x)+min(x))/2;
    hardIronOffset(2) = (max(y)+min(y))/2;
    hardIronOffset(3) = (max(z)+min(z))/2;

    disp('Calculated hard iron offset: ');
    disp(num2str(hardIronOffset));

    txyzCalibrated = [ ...
        x - hardIronOffset(1), ...
        y - hardIronOffset(2), ...
        z - hardIronOffset(3)];
    txyzCalibrated = [txyz(:,1) txyzCalibrated(:, 1:3)];

end