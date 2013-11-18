function [ rotatedVector ] = rotate( vector, rollX, pitchY, yawZ )
%ROTATE Rotates a vector (in yaw-pitch-roll order)
%   Rotates a vector using roll/pitch/yaw angles in radians

    Rx = [1 0 0;
        0 cos(rollX) -sin(rollX);
        0 sin(rollX) cos(rollX)];

    Ry = [cos(pitchY) 0 sin(pitchY);
        0 1 0;
        -sin(pitchY) 0 cos(pitchY)];
    
    Rz = [cos(yawZ) -sin(yawZ) 0;
        sin(yawZ) cos(yawZ) 0;
        0 0 1];
    
    v = vector;
    
    % transpose if needed
    if isrow(vector)
        v = vector';
    end
    
    % rotate matrix
    rotatedVector = Rz*Ry*Rx*v;
    
    % de-transpose
    if isrow(vector)
        rotatedVector = rotatedVector';
    end
    
end

