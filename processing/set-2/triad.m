function [ DCM ] = triad( a, m, A, M )
%TRIAD Implements TRIAD algorithm for Direction Cosine Matrix estimation

    if ~exist('A', 'var')
        A = [0 0 1];
        M = [1 0 0];
    end
    
    % normalize body frame axes
    an = a / norm(a);
    mn = m / norm(m);
    
    % normalize reference frame axes
    An = A / norm(A);
    Mn = M / norm(M);
    
    % after normalisation, an (positive up) is the the Z axis
    z = an;             % Z is already normalised
    
    % after normalisation, mn is identical to the X axis
    % Note that Z and X are not necessarily orthogonal due to the fact
    % that X points straight towards magnetic north.
    x = mn;             % X is already normalised

    % calculate global Y by crossing X and Z
    y = cross(z, m);
    y = y/norm(y);
    
    % re-generate X from Z and Y
    % Z and X are orthogonal afterwards.
    x = cross(y, z);    % Y is normalised because of Z and Y
    x = x/norm(x);
    
    % re-generate Z from X and Y
    % This would not be necessary if local z is already pointing up.
    z = cross(x, y);
    z = z/norm(z);
    
    % Generate reference system
    Z = An;
    X = Mn;
    Y = cross(Z, X);
    
    % generate direction cosine matrix.
    % this matrix contains all the rotation angles that have been applied
    % beforehand by the rotate() method. (This essentially IS the rotation
    % matrix.)
    % Not all fields are required for angle detection
    % and should be omitted for performance.
    %{
    DCM = [ ...
        dot(x, X),  dot(y, X),  dot(z, X);
        dot(x, Y),  dot(y, Y),  dot(z, Y);
        dot(x, Z),  dot(y, Z),  dot(z, Z);
        ];  
    %}
    
    % Calculate DCM
    DCM = [x; y; z] * [X; Y; Z];
    
end

