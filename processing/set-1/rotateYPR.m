function [ rotatedVector ] = rotateYPR( vector, rollX, pitchY, yawZ )
%ROTATE Rotates a vector (in yaw-pitch-roll order)
%   Rotates a vector using roll/pitch/yaw angles in radians

    %{
    syms rollX pitchY yawZ vector

    Rx = [1 0 0;
        0 cos(rollX) -sin(rollX);
        0 sin(rollX) cos(rollX)];

    Ry = [cos(pitchY) 0 sin(pitchY);
        0 1 0;
        -sin(pitchY) 0 cos(pitchY)];
    
    Rz = [cos(yawZ) -sin(yawZ) 0;
        sin(yawZ) cos(yawZ) 0;
        0 0 1];

    rotatedVector = Rz*Ry*Rx*v

    Rzyx = [ cos(pitchY)*cos(yawZ), -(cos(rollX)*sin(yawZ) - cos(yawZ)*sin(pitchY)*sin(rollX)),  (sin(rollX)*sin(yawZ) + cos(rollX)*cos(yawZ)*sin(pitchY));
             cos(pitchY)*sin(yawZ),  (cos(rollX)*cos(yawZ) + sin(pitchY)*sin(rollX)*sin(yawZ)), -(cos(yawZ)*sin(rollX) - cos(rollX)*sin(pitchY)*sin(yawZ));
                      -sin(pitchY),                                     cos(pitchY)*sin(rollX),                                     cos(pitchY)*cos(rollX)]
    %}
    
    Rx = [1 0 0;
        0 cos(rollX) -sin(rollX);
        0 sin(rollX) cos(rollX)];

    Ry = [cos(pitchY) 0 sin(pitchY);
        0 1 0;
        -sin(pitchY) 0 cos(pitchY)];
    
    Rz = [cos(yawZ) -sin(yawZ) 0;
        sin(yawZ) cos(yawZ) 0;
        0 0 1];

    Rzyx = Rx*Ry*Rz
    
    v = vector;
    
    % transpose if needed
    if isrow(vector)
        v = vector';
    end
    
    % rotate matrix
    rotatedVector = Rzyx * v;
    
    % de-transpose
    if isrow(vector)
        rotatedVector = rotatedVector';
    end
    
end

