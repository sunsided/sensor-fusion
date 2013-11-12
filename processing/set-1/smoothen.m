function [filtered] = smoothen(data, windowSize)
    if ~exist('windowSize', 'var')
        windowSize = 5;
    end

    % create 1D smoothening kernel
    kernel = ones(1, windowSize);
    
    % filter with kernel
    filtered = filter(kernel, windowSize, data);
end