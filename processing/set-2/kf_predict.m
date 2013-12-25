function [x, P] = kf_predict(x, u, A, B, P, Q)

    if nargin == 3
        
        % predict state
        x = A*x;
        
        % predict covariance
        P = A*P*A';
    
    elseif nargin == 6
    
        % predict state
        x = A*x + B*u;
        
        % predict covariance
        P = A*P*A' + B*Q*B';
    
    else
        error('kf_predict:argument_check', 'Wrong number of input arguments');
    end
    
end