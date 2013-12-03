function [ varargout ] = lerpTimeSeries( varargin )
%LERPDATA Performs a linear interpolations on multiple vector time series

    if nargin < 1
        error('lerpVectors:argumentCheck', 'Not enough input arguments.');
    end

    if nargin ~= nargout
        error('lerpVectors:argumentCheck', 'Number of output arguments must match number of input arguments.');
    end
    
    % merge time vectors
    t = varargin{1}.Time;
    for n=2:nargin
        t = union(t, varargin{n}.Time);
    end
    
    % resample all time series
    for n=1:nargin
        ts = varargin{n};
        
        % to aid extrapolation, pad at start and end
        first = ts.Data(1, :);
        last = ts.Data(end, :);
        ts = addsample(ts, 'Data', first, 'Time', t(1));
        ts = addsample(ts, 'Data', last,  'Time', t(end));
        
        % resample the timeseries
        varargout{n} = resample(ts, t);
    end
        
end

