function [q] = quaternionFromAngularVelocity(w, dT)

    % Prepare quaternion components
    qw = 1;    qx = 0;    qy = 0;    qz = 0;

    % Creates a quaternion from an angular velocity pseudovector
    % Axes are x,y,z (roll, pitch, yaw)
    % Values are given in rad/sec
    % dT is the time differential
    x = w(1) * dT;    y = w(2) * dT;    z = w(3) * dT;

    % Convert angular velocity
    % http://stackoverflow.com/a/12054031/195651

    angle = sqrt(x*x + y*y + z*z);
    s = sin(0.5 * angle) / angle;

    if angle > 0
        qw = cos(0.5 * angle);
        qx = x * s;
        qy = y * s;
        qz = z * s;
    end

    % Assemble quaternion
    q = [qw qx qy qz];
end