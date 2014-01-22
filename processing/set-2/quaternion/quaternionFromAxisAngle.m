function [q] = quaternionFromAxisAngle(axis, angle)

    ax = axis(1);    ay = axis(2);    az = axis(3);

    qw = cos(angle/2);
    qx = ax * sin(angle/2);
    qy = ay * sin(angle/2);
    qz = az * sin(angle/2);
    
    q = [qw qx qy qz];

end