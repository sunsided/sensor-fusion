function [M,S] = affine_rotation_y(theta)

    M = [cos(theta) 0  sin(theta)  0;
         0          1  0           0;
        -sin(theta) 0  cos(theta)  0;
         0 0           0           1];

    if nargout == 2
        syms cosy siny;
        S = [cosy 0  siny  0;
             0    1  0     0;
            -siny 0  cosy  0;
             0 0  0        1];     
    end
     
end