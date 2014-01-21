function [q] = quatConj(x)

    q(1,1) = x(1);
    q(2,1) = -x(2);
    q(3,1) = -x(3);
    q(4,1) = -x(4);

end