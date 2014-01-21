function [q] = quatFromDCM(DCM)

    % builds a quaternion from a direction cosine matrix
    % http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html

    q = sqrt( 0.25 * [1  1 -1 -1;
                      1 -1  1 -1;
                      1 -1 -1  1;
                      1  1  1  1] * [DCM(1,1); DCM(2,2); DCM(3,3); 1] );
     
    q(1) = abs(q(1));
    q(2) = sign(DCM(3,2) - DCM(2,3)) * abs(q(2));
    q(3) = sign(DCM(1,3) - DCM(3,1)) * abs(q(3));
    q(4) = sign(DCM(2,1) - DCM(1,2)) * abs(q(4));

end