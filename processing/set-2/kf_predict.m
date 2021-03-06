function [x, P] = kf_predict(x, A, P, lambda, u, B, Q)

    if ~exist('lambda', 'var')
        lambda = 0.999;
    end

    if ~exist('B', 'var') && exist('u', 'var') && ismatrix(u)
        B = eye(size(P));
        Q = u;
        u = zeros(length(Q), 1);
    end
    
    if nargin == 3 || nargin == 4
        
        % predict state
        x = A*x;
        
        % predict covariance
        P = A*P*A' * 1/(lambda^2);
    
    elseif nargin == 5 || nargin == 7
    
        % predict state
        x = A*x + B*u;
        
        % predict covariance
        P = A*P*A' * 1/(lambda^2) + B*Q*B';
    
    else
        error('kf_predict:argument_check', 'Wrong number of input arguments');
    end
    
end