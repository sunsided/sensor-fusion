function [w] = quaternionToAngularVelocity(q1, q0, T)

    % q1 is the current orientation
    % q0 is the previous orientation
    % T is the time separating the orientations
    
    % http://lost-found-wandering.blogspot.de/2011/09/revisiting-angular-velocity-from-two.html
    
    % derive quaternion that transforms q0 into q1
    r = quaternionMul(q1, quaternionInv(q0));
    
    % from r, find the angle between the quaternions
    theta = 2*acos(r(1));
    
    % derive angular velocity from vectorial component
    v = r(2:4);
    w = (theta/T) * v/norm(v);

end