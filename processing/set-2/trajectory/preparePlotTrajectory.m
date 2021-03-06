function preparePlotTrajectory(figureTitle)

    if ~exist('figureTitle', 'var')
        figureTitle = 'Trajectory';        
    end

    global trajectoryPlotHandle trajectoryAxis

    % define base colors
    lineColor(1, :) = [1 0.25 0] * 0.75;
    lineColor(2, :) = [1 1 1] * 0.75;
    lineColor(3, :) = [1 1 1];
    axesColor = [0.473 0.473 0.473];
    plotBackground = [0.15 0.15 0.15];
    titleColor = [1 1 1];
    
    trajectoryFigureHandle(1) = figure('Name', figureTitle, ...
        'NumberTitle', 'off', ...
        'Color', [0.027 0.211 0.259] ...
        );

    trajectoryAxis(1) = subplot(1, 1, 1, ...
        'Parent', trajectoryFigureHandle(1), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'ZGrid', 'on', ...
        'ZColor', axesColor, ...
        'Color', plotBackground ...
        );

    trajectoryPlotHandle(1) = line(NaN, NaN, NaN, ...
        'Parent', trajectoryAxis, ...
        'Color', lineColor(1,:), ...
        'LineWidth', 1); hold on;
    trajectoryPlotHandle(2) = line(NaN, NaN, NaN, ...
        'Parent', trajectoryAxis, ...
        'Color', lineColor(2,:), ...
        'LineStyle', ':', ...
        'LineWidth', 1);
    trajectoryPlotHandle(3) = line(NaN, NaN, NaN, ...
        'Parent', trajectoryAxis, ...
        'Color', lineColor(3,:), ...
        'LineStyle', 'none', ...
        'Marker', '.', ...
        'MarkerSize', 20, ...
        'LineWidth', 1);
    view(3);
    width = 1.25;
    xlim([-width width]); ylim([-width width]); zlim([-width width]);
    xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
    grid on;
    axis equal;
    
    camproj('perspective');
    view(3);
    rotate3d;

end