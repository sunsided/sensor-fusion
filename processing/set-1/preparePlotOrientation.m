function preparePlotOrientation()

    global coordSysPlotHandle orientationPlotHandle

    % define base colors
    lineColor(1, :) = [1 0.25 0]; % x axis
    lineColor(2, :) = [0.5 1 0]; % y axis
    lineColor(3, :) = [0 0.5 1]; % z axis
    axesColor = [0.473 0.473 0.473];
    plotBackground = [0.15 0.15 0.15];
    titleColor = [1 1 1];

    figureHandle(1) = figure('Name', 'Coordinate System', ...
        'NumberTitle', 'off', ...
        'Color', [0.027 0.211 0.259] ...
        );

    axisCoordinate(1) = subplot(1, 1, 1, ...
        'Parent', figureHandle(1), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'ZGrid', 'on', ...
        'ZColor', axesColor, ...
        'Color', plotBackground ...
        );

    coordSysPlotHandle(1) = line(NaN, NaN, NaN, ...
        'Color', lineColor(1,:), ...
        'LineWidth', 3); hold on;
    coordSysPlotHandle(2) = line(NaN, NaN, NaN, ...
        'Color', lineColor(2,:), ...
        'LineWidth', 3);
    coordSysPlotHandle(3) = line(NaN, NaN, NaN, ...
        'Color', lineColor(3,:), ...
        'LineWidth', 3);
    coordSysPlotHandle(4) = line(NaN, NaN, NaN, ...
        'Color', lineColor(3,:), ...
        'LineWidth', 1);
    coordSysPlotHandle(5) = line(NaN, NaN, NaN, ...
        'Color', lineColor(1,:), ...
        'LineWidth', 1);
    view(3);
    xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);
    grid on;
    axis equal;

    camproj('perspective');
    view(-90, 15);
    rotate3d;

    % Prepare Rotation Plots
    figureHandle(2) = figure('Name', 'Orientation', ...
        'NumberTitle', 'off', ...
        'Color', [0.027 0.211 0.259] ...
        );

    % 3D
    orientation(1) = subplot(3, 3, [2 3 5 6 8 9], ...
        'Parent', figureHandle(2), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'Color', plotBackground ...
        );

    orientationPlotHandle(1) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
    orientationPlotHandle(2) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
    xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
    axis square; grid on;

    camproj('perspective');
    view(-90, 15);
    rotate3d;

    % front
    orientation(2) = subplot(3, 3, 1, ...
        'Parent', figureHandle(2), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'Color', plotBackground ...
        );

    orientationPlotHandle(3) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
    orientationPlotHandle(4) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

    title('front view (X/Z)', 'Color', titleColor);
    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
    xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
    axis square; grid on;

    view(0, 0);

    % top
    orientation(3) = subplot(3, 3, 4, ...
        'Parent', figureHandle(2), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'Color', plotBackground ...
        );

    orientationPlotHandle(5) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
    orientationPlotHandle(6) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

    title('top view (X/Y)', 'Color', titleColor);
    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
    xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
    axis square; grid on;

    view(-90, 90);

    % left
    orientation(4) = subplot(3, 3, 7, ...
        'Parent', figureHandle(2), ...
        'XGrid', 'on', ...
        'XColor', axesColor, ...
        'YGrid', 'on', ...
        'YColor', axesColor, ...
        'Color', plotBackground ...
        );
    axis square;

    % Prepare plot
    orientationPlotHandle(7) = fill3(NaN, NaN, NaN, [1 0 0]); hold on;
    orientationPlotHandle(8) = fill3(NaN, NaN, NaN, [0.5 0 0]); hold on;

    title('side view (Y/Z)', 'Color', titleColor);
    xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
    xlabel('x', 'Color', axesColor); ylabel('y', 'Color', axesColor); zlabel('z', 'Color', axesColor);
    axis square; grid on;

    view(-90, 0);

end