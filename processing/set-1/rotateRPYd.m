function [ rotatedVector ] = rotateRPYd( vector, rollX, pitchY, yawZ )
%ROTATE Rotates a vector (roll-pitch-yaw order in degree)
%   Rotates a vector using roll/pitch/yaw angles in degree

    scale = pi/180;
    rotatedVector = rotateRPY(vector, rollX*scale, pitchY*scale, yawZ*scale);
    
end

