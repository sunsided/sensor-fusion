function plotTrajectory(xyz)

    global trajectoryPlotHandle
    
    % Draw point
    x = [get(trajectoryPlotHandle(1), 'XData') xyz(1)];
    y = [get(trajectoryPlotHandle(1), 'YData') xyz(2)];
    z = [get(trajectoryPlotHandle(1), 'ZData') xyz(3)];
    set(trajectoryPlotHandle(1), 'XData', x, 'YData', y, 'ZData', z);
    
    % Draw line
    x = [0 xyz(1)];
    y = [0 xyz(2)];
    z = [0 xyz(3)];
    set(trajectoryPlotHandle(2), 'XData', x, 'YData', y, 'ZData', z);
    
    drawnow;
end