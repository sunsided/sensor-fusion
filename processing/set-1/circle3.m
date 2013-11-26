function [points] = circle3(N, center, noiseVariance)
% CIRCLE3 Creates a sphere. Yeah ... I know.

    if ~exist('N', 'var')
        N = 100;
    end

    if ~exist('center', 'var')
        center = [0, 0, 0];
    end

    if ~exist('noiseVariance', 'var')
        noiseVariance = 0;
    end

    points = zeros(N, 3);
    
    r = 1;
    for i=1:N
        azimuth = rand()*2*pi;
        elevation = rand()*2*pi;
        [x, y, z] = sph2cart(azimuth, elevation, r);
        points(i, :) = [x, y, z] + sqrt(noiseVariance)*randn(1, 3) + center;
    end
    
end