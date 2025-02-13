function pcDisplay(varargin)
%Overrides pcshow
    % Wrapper for pcshow to include default AxesVisibility and axis labels

    % Call the original pcshow function
    pcshow(varargin{:}, 'AxesVisibility', 'on');
    
    % Add axis labels
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end