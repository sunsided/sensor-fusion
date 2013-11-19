function [M,S] = affine_translation(translation)

    M = [1 0 0 translation(1);
         0 1 0 translation(2);
         0 0 1 translation(3);
         0 0 0 1];

    syms tx ty tz;
    S = [1 0 0 tx;
         0 1 0 ty;
         0 0 1 tz;
         0 0 0 1];     
     
end