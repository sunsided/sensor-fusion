function plotTrajectory(xyz)

    global trajectoryPlotHandle trajectoryAxis
    
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
    
    % Draw point
    x = xyz(1);
    y = xyz(2);
    z = xyz(3);
    set(trajectoryPlotHandle(3), 'XData', x, 'YData', y, 'ZData', z);
    
    msg = sprintf('%+1.3f %+1.3f %+1.3f', x, y, z);
    title(trajectoryAxis, msg, 'Color', [1 1 1]);
    xlabel(trajectoryAxis, ['x: ' num2str(x)]);
    ylabel(trajectoryAxis, ['y: ' num2str(y)]);
    zlabel(trajectoryAxis, ['z: ' num2str(z)]);
    
    drawnow;
end