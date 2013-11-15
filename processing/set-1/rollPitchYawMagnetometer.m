function [rpy] = rollPitchYawMagnetometer(mag)
    % ROLLPITCH Calculates roll and pitch angles from measured magnetometer
    %           data
    %
    %   Parameters
    %   mag = 3D vector of component magnetometer data [x; y; z]
    %         where z is interpreted as up vector, that is, if the
    %         magnetometer measures maximum field downwards, then z is positive.
    %         
    %
    %   Return values
    %   vector [roll; pitch; yaw] in radians
    
    % Get normalized component vectors
    invn = 1/norm(mag);
    x =  [1; 0; 0] * mag(1) * invn;
    y =  [0; 1; 0] * mag(2) * invn;
    z =  [0; 0; 1] * mag(3) * invn;

    % Reference vector
    xref = [1; 0; 0];
    yref = [0; 1; 0];
    zref = [0; 0; 1];

    % Prepare result
    rpy = [NaN; NaN; NaN];
    
    % Get y-z component angle (roll)
    zy_cross = cross(zref, y);
    yangle   = zy_cross(1);
    rpy(1)    = asin(yangle);
    %{
    if mag(3) < -pi/2
       rpy(1) = pi - rpy(1);
    end
    %}
        
    % Get x-z component angle (pitch)
    xz_cross = cross(x, zref);
    xangle   = xz_cross(2);
    rpy(2)     = asin(xangle);
    %{
    if mag(3) < 0
       rpy(2) = -rpy(2);
    end
    %}
    
    % Get y-x component angle (yaw)
    yx_cross = cross(yref, x)
    zangle   = yx_cross(3);
    rpy(3)     = asin(zangle);
    %{
    if mag(3) < 0
       rpy(2) = -rpy(2);
    end
    %}
end