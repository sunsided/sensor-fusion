function [w] = quaternionToAngularVelocity2(q1, q0, T)

    % Calculates angular velocities from two quaternions
    % using interpolation on the quaternion hypersphere.

    % q1 is the current orientation
    % q0 is the previous orientation
    % T is the time separating the orientations
    
    % http://lost-found-wandering.blogspot.de/2011/04/angular-velocity-from-two-known.html
    
    % derive the angle between both quaternions on the hypersphere
    theta = acos(quaternionDot(q0, q1));
    if theta < 0 || theta > pi/2
        q1 = -q1;
    end

    % differentiate SLERP function
    % dq = (theta*cos(T*theta)/sin(theta))*q1 - (theta*cos((1-T)*theta)/sin(theta))*q0;
    % solve for T = 0
    dq = theta/sin(theta) * (q1 - cos(theta)*q0);
    
    % extract angular velocities
    qomega = 2/T * (quaternionMul(dq, quaternionInv(q0)));
    w = qomega(2:4);
end