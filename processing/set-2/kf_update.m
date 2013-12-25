function [x, P] = kf_update(x, z, P, H, R)

    % measurement residuals (innovation)
    y = z - H*x;
    
    % residual (innovation) covariance
    S = H*P*H' + R;
    
    % Kalman gain
    K = P*H' / S;
    
    % update state prediction
    x = x + K*y;
    
    % update covariance matrix
    P = (eye(size(P)) - K*H)*P;
    
end