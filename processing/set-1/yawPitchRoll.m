function [azimuthYaw, elevationPitch, roll, DCM, coordinateSystem] = yawPitchRoll(accelerometer, magnetometer)

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
    % Note that Z and X are not necessarily orthogonal due to the fact
    % that X points straight towards magnetic north.
    X = mn;             % X is already normalised

    % calculate global Y by crossing X and Z
    Y = cross(X, Z);
    Y = Y/norm(Y);
    
    % re-generate X from Z and Y
    % Z and X are orthogonal afterwards.
    X = cross(Z, Y);    % Y is normalised because of Z and Y
    X = X/norm(X);
    
    % re-gererate Z from X and Y
    Z = cross(X, Y);
    Z = Z/norm(Z);
    
    % save coordinate system
    coordinateSystem = [X; Y; Z];
    
    % generate direction cosine matrix.
    % this matrix contains all the rotation angles that have been applied
    % beforehand by the rotate() method. (This essentially IS the rotation
    % matrix.)
    % Not all fields are required and should be omitted for performance.
    DCM = [ ...
        dot(X, x),  dot(Y, x),  dot(Z, x);
        dot(X, y),  dot(Y, y),  dot(Z, y);
        dot(X, z),  dot(Y, z),  dot(Z, z);
        ];

    % extract angles
    cosPitchYsinRollX = -DCM(2,3);
    cosPitchYcosRollX =  DCM(3,3);
    rollX = -atan2(cosPitchYsinRollX, cosPitchYcosRollX) * 180/pi;

    cosPitchYcosYawZ =  DCM(1,1);
    cosPitchYsinYawZ = -DCM(1,2);
    yawZ = -atan2(cosPitchYsinYawZ, cosPitchYcosYawZ) * 180/pi;

    sinYawZ = sind(yawZ);
    cosYawZ = cosd(yawZ);
    sinPitchY = DCM(1,3);

    %pitchY1 = -atan2(sinPitchY, cosPitchYsinYawZ/sinYawZ) * 180/pi
    %pitchY2 = -atan2(sinPitchY*sinYawZ, cosPitchYsinYawZ) * 180/pi
    pitchY3 = -atan2(sinPitchY, cosPitchYcosYawZ/cosYawZ) * 180/pi;
    %pitchY4 = -atan2(sinPitchY*cosYawZ, cosPitchYcosYawZ) * 180/pi
    %pitchY4 = -atan2(sinPitchY, cosPitchYsinRollX/sind(rollX)) * 180/pi
    %pitchY5 = -atan2(sinPitchY*sind(rollX), cosPitchYsinRollX) * 180/pi
    %pitchY6 = -atan2(sinPitchY, cosPitchYcosRollX/cosd(rollX)) * 180/pi
    pitchY7 = -atan2(sinPitchY*cosd(rollX), cosPitchYcosRollX) * 180/pi;
    if rollX >= 90 || rollX <= -90
        pitchY = pitchY3;
    else
        pitchY = pitchY7;
    end

    %% Fetch angles

    % heading in the X/Y plane
    azimuthYaw = yawZ;

    % elevation in the X/Z plane
    elevationPitch = pitchY;

    % roll in the Z/Y plane
    roll = rollX;

end