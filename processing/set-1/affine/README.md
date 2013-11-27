# Affine Transformations #

Functions for numeric and symbolic creation of affine transformation matrices.

Each function returns a numerical solution, e.g. for given `theta`

	M = [1 0           0           0;
         0 cos(theta) -sin(theta)  0;
         0 sin(theta)  cos(theta)  0;
         0 0           0           1];

as well as a symbolic variant if requested, e.g.

	syms cosx sinx;
    S = [1 0     0     0;
    	 0 cosx -sinx  0;
         0 sinx  cosx  0;
         0 0     0     1];