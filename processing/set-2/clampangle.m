function [ angles ] = clampangle( angles )
%CLAMPANGLE Clamps angles to a range of -180..180

    while true
        [r,c] = find(angles > 180);
        if isempty(r); break; end;
        angles(r,c) = angles(r,c) - 360;
    end

    while true
        [r,c] = find(angles < -180);
        if isempty(r); break; end;
        angles(r,c) = angles(r,c) + 360;
    end

end

