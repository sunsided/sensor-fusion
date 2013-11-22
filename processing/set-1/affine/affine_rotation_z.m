function [M,S] = affine_rotation_z(theta)

    M = [cos(theta) -sin(theta) 0 0;
         sin(theta)  cos(theta) 0 0;
         0           0          1 0;
         0           0          0 1];

    if nargout == 2
        syms cosz sinz;
        S = [cosz -sinz 0 0;
             sinz  cosz 0 0;
             0     0    1 0;
             0     0    0 1];     
    end
     
end