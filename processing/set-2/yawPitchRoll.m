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
        ]'; % note the inverse to represent the local frame

    % extract angles
    % see: William Premerlani, "Computing Euler Angles from Direction Cosines"
    %      Indices differ from his version due to the transposed DCM.
    pitchY = -asind(DCM(3, 1));
    rollX = atan2d(DCM(3, 2), DCM(3, 3));
    yawZ = atan2d(DCM(2, 1), DCM(1, 1));

    %% Fetch angles

    % heading in the X/Y plane
    azimuthYaw = yawZ;

    % elevation in the X/Z plane
    elevationPitch = pitchY;

    % roll in the Z/Y plane
    roll = rollX;

end