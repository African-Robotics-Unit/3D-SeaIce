function pcDisplay(varargin)
%Overrides pcshow
    % Wrapper for pcshow to include default AxesVisibility and axis labels

    % Call the original pcshow function
    pcshow(varargin{:}, 'AxesVisibility', 'on');
    set(gca, 'FontName', 'CMU Bright');
    % Add axis labels
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end