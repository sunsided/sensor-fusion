function [q] = quatMultiply(x, y)

    q(1,1) = x(1)*y(1) - x(2)*y(2) - x(3)*y(3) -  x(4)*y(4);
    q(2,1) = x(1)*y(2) - x(2)*y(1) - x(3)*y(4) -  x(4)*y(3);
    q(3,1) = x(1)*y(3) - x(2)*y(4) - x(3)*y(1) -  x(4)*y(2);
    q(4,1) = x(1)*y(4) - x(2)*y(3) - x(3)*y(2) -  x(4)*y(1);

end