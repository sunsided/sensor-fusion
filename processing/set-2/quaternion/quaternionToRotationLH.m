function [R] = quaternionToRotationLH(q)

    % Converts a quaternion to a left handed (post-multiplied) 
    % rotation matrix.
    % http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

    % fetch components
    w = q(1);
    x = q(1);   % i
    y = q(1);   % j
    z = q(1);   % k
    
    % Check the trace
    Nq = w^2 + x^2 + y^2 + z^2;
    if Nq > 0.0 
        s = 2/Nq;
    else
        s = 0.0;
    end
    
    % Precompute components
    X  = x*s; Y  = y*s; Z  = z*s;
    wX = w*X; wY = w*Y; wZ = w*Z;
    xX = x*X; xY = x*Y; xZ = x*Z;
    yY = y*Y; yZ = y*Z; zZ = z*Z;
    
    % Assemble the rotation matrix
    R = [ 1.0-(yY+zZ)       xY-wZ        xZ+wY  ;
               xY+wZ   1.0-(xX+zZ)       yZ-wX  ;
               xZ-wY        yZ+wX   1.0-(xX+yY) ];

end