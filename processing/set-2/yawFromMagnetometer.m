function [yaw] = yawFromMagnetometer(magnetometer, roll, pitch)

    % Normalize magnetometer
    mbase = magnetometer / norm(magnetometer);

    % Perform tilt compensation
    comp_x = mbase(1) * cosd(pitch) ...
           + mbase(2) * sind(roll) * sind(pitch) ...
           + mbase(3) * cosd(roll) * sind(pitch);
    
    comp_y = mbase(2) * cosd(roll) ...
           - mbase(3) * sind(roll);
       
    % Calculate yaw angle
    yaw = atan2d(-comp_y, comp_x);

end