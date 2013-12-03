function [ a, b ] = lerpVectors( a, b )
%LERPDATA Performs a linear interpolations on two vector series

    % merge the timestamps and sort them
    t = union(a(:,1), b(:,1));

    % to aid extrapolation, pad at start and end
    a = [t(1), a(1,2:4);
        a(:,:);
        t(end), a(end,2:4)];
    
    b = [t(1), b(1,2:4);
        b(:,:);
        t(end), b(end,2:4)];
    
    % create timeseries from the time vectors
    ts1 = timeseries(a(:,2:4), a(:,1), 'Name', 'a');
    ts2 = timeseries(b(:,2:4), b(:,1), 'Name', 'b');
    
    % resample the time series
    ts1 = resample(ts1, t);
    ts2 = resample(ts2, t);
    
    % extract data
    a = [ts1.Time, ts1.Data];
    b = [ts2.Time, ts2.Data];
    
end

