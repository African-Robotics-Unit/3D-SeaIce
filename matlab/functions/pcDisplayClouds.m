function pcDisplayClouds(pcArray)
    % Validate input
    if length(pcArray) > 10
        error('pcDisplayClouds only supports up to 10 point clouds.');
    end

    % Define distinct colors (10 max)
    colors = [
        1 0 0;     % Red
        0 1 0;     % Green
        0 0 1;     % Blue
        1 1 0;     % Yellow
        1 0 1;     % Magenta
        0 1 1;     % Cyan
        0.5 0.5 0.5; % Gray
        1 0.5 0;   % Orange
        0.5 0 1;   % Purple
        0 0.5 0.5  % Teal
    ];

    figure;
    hold on;
    grid on;
    axis equal;
    title('Multiple Point Clouds');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    for i = 1:length(pcArray)
        pc = pcArray(i);
        if ~isa(pc, 'pointCloud')
            error('Each element in the input must be a pointCloud object.');
        end

        % Create a color matrix matching number of points in this cloud
        c = repmat(colors(i,:), pc.Count, 1);

        % Use pcshow with custom color
        pcshow(pc.Location, c,"AxesVisibility","on");
    end

    legendStrings = arrayfun(@(i) sprintf('Cloud %d', i), 1:numel(pcArray), 'UniformOutput', false);
    legend(legendStrings, 'Location', 'best');
    hold off;
end