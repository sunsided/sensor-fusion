% Drift correction
function [p, driftcorrected] = determineDrift(ts)

    N = size(ts.Data, 2);
    p = NaN(N, 2);
    
    for n=1:N
        p(n,:) = polyfit(ts.Time, ts.Data(:,n), 1);
    end

    if nargout == 2
        driftcorrected = timeseries(ts);
        for n=1:N
            driftcorrected.Data(:,n) = ts.Data(:,n) - (ts.Time * p(n,1) + p(n,2));
        end
    end
    
end
    

