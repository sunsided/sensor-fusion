function [s] = quaternionDot(a, b)

    as = a(1);    ax = a(2);    ay = a(3);    az = a(4);
    bs = b(1);    bx = b(2);    by = b(3);    bz = b(4);
   
    s = as*bs + ax*bx + ay*by + az*bz;
    
end