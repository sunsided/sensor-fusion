function [M,S] = affine_scale(sx, sy, sz)

    M = [sx 0  0  0;
         0  sy 0  0;
         0  0  sz 0;
         0  0  0  1];
     
    syms s;
    S = [s 0 0 0;
         0 s 0 0;
         0 0 s 0;
         0 0 0 1];

end