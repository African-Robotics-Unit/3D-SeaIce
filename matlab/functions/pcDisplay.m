function pcDisplay(varargin)
%Overrides pcshow
    % Wrapper for pcshow to include default AxesVisibility and axis labels

    % Call the original pcshow function
    pcshow(varargin{:}, 'AxesVisibility', 'on');
    set(gca, 'FontName', 'CMU Bright');
    view([-73.8056026442575 28.8574342589771]);
ax = gca; % Get the current axes handle
fig = gcf; % Get the current figure handle
% Set axes properties
ax.Color = 'white'; % Axes background
ax.XColor = 'black'; % X-axis color
ax.YColor = 'black'; % Y-axis color
ax.ZColor = 'black'; % Z-axis color
ax.GridColor = 'black'; % Grid line color
ax.MinorGridColor = 'black'; % Minor grid line color (if enabled)

% Optional: Enable grid if not already on
grid on;
% Set background color to white
fig.Color = 'white';
    % Add axis labels
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end