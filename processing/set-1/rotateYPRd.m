function [ rotatedVector ] = rotateYPRd( vector, rollX, pitchY, yawZ )
%ROTATE Rotates a vector (yaw-pitch-roll order in degree)
%   Rotates a vector using roll/pitch/yaw angles in degree

    scale = pi/180;
    rotatedVector = rotateYPR(vector, rollX*scale, pitchY*scale, yawZ*scale);
    
end

