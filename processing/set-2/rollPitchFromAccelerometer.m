function [roll, pitch, roll_error] = rollPitchFromAccelerometer(accelerometer, mu)

    % Normalize and invert acceleration vector 
    % to measure up instead of gravity.
    zbase = -accelerometer / norm(accelerometer);

    % Find roll from accelerometer
    roll_unstable  = atan2d(zbase(2), zbase(3));

    if ~exist('mu', 'var')
        mu = 0.01;
    end
        
    % Calculate stable approximation of roll angle
    roll_stable  = atan2d(zbase(2), sign(zbase(3)) * sqrt(mu*zbase(1)^2 + zbase(3)^2));
    
    % Select flavor.
    roll = roll_stable;
    
    % Calculate error of roll angle to unstable, exact value
    roll_error = roll_stable - roll_unstable;
    
    % Note that Pitch is defined to be [-90� .. 90�]
    %
    % We are negating here because of the inverted sign
    % of the accelerometer reading
    pitch = -atan2d(zbase(1), sqrt(zbase(2)^2 + zbase(3)^2));
    
    % This is an alternate was of computing pitch, as per
    % http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
    %pitch = -atan2d(zbase(1), zbase(2)*sind(qroll2) + zbase(3)*cosd(qroll2));
       
    % Use this fix is applied to prevent the 180� roll angle inversion
    % when pitch would be becoming smaller than -90� or larger than 90�.
    % (See integrated gyroscope readings for the effect.)
    % Note, however, that the uncorrected value is the true reading, 
    % since havinga pitch angle lower than -90� or larger than 90� 
    % results in the inverse position, i.e. flipped roll and heading.
    % (To make it clear: Inverting would be required for Yaw, too!)
    %{
    if zbase(3) < 0
       roll = roll - 180;
    end
    %}    
    
end