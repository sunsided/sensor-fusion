function [M, S] = affine_rotation_x(theta)

    M = [1 0           0           0;
         0 cos(theta) -sin(theta)  0;
         0 sin(theta)  cos(theta)  0;
         0 0           0           1];

    if nargout == 2
        syms cosx sinx;
        S = [1 0     0     0;
             0 cosx -sinx  0;
             0 sinx  cosx  0;
             0 0     0     1];
    end
     
end