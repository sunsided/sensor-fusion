function [azimuthYaw, elevationPitch, roll] = yawPitchRoll(accelerometer, magnetometer)

    %% Define local coordinate system

    x = [1; 0; 0];
    y = [0; 1; 0];
    z = [0; 0; 1];

    %% Prepare measurements

    an = accelerometer / norm(accelerometer);
    mn = magnetometer / norm(magnetometer);

    % after normalisation, an (positive up) is the the Z axis
    Z = an;             % Z is already normalised

    % after normalisation, mn is identical to the X axis
    X = mn;             % X is already normalised

    % calculate global Y by crossing X and Z
    Y = cross(Z, X);    % Y is already normalised

    %% Calculate axis cross products

    xX = cross(x, X);
    xY = cross(x, Y);
    xZ = cross(x, Z);

    yX = cross(y, X);
    yY = cross(y, Y);
    yZ = cross(y, Z);

    zX = cross(z, X);
    zY = cross(z, Y);
    zZ = cross(z, Z);

    %% Calculate angles

    azimuthRad = -atan2(xX(3), xY(3));
    azimuthYaw = azimuthRad * 180/pi;

    elevationRad = -atan2(xX(3), xZ(2));
    elevationPitch = elevationRad * 180/pi;

    rollRad = -atan2(xY(2), xY(3));
    roll = rollRad * 180/pi;

end