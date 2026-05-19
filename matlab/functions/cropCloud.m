function [pcCropped,roi] = cropCloud(pcloud,xcrop,ycrop,zcrop)
% CROPCLOUD Crop a point cloud within specified ranges.
%
%   pcCropped = CROPCLOUD(pcloud) crops the point cloud using the default 
%   range of [-inf, inf] for all dimensions (x, y, z).
%
%
%   pcCropped = CROPCLOUD(pcloud, xcrop, ycrop, zcrop) crops the point cloud 
%   using the specified ranges for the x, y, and z dimensions.
%
%   Inputs:
%       pcloud - A point cloud object.
%       xcrop  - 2-element vector [xmin, xmax] for the x-dimension cropping range.
%                Optional, default is [-inf, inf].
%       ycrop  - 2-element vector [ymin, ymax] for the y-dimension cropping range.
%                Optional, default is [-inf, inf].
%       zcrop  - 2-element vector [zmin, zmax] for the z-dimension cropping range.
%                Optional, default is [-inf, inf].
%
%   Output:
%       pcCropped - Cropped point cloud object.
%       roi       - roi used for the crop
%   Example:
%       [pcCropTest,roiTest]=cropCloud(pcRotated,[],[1,1.5],[0,0.5]);

 % Set default values
    defaultVal = [-inf, inf];

    % Check number of arguments and set defaults if necessary
    if nargin < 2 || isempty(xcrop)
        xcrop = defaultVal;
    end
    if nargin < 3 || isempty(ycrop)
        ycrop = defaultVal;
    end
    if nargin < 4 || isempty(zcrop)
        zcrop = defaultVal;
    end

    % Ensure each crop parameter is a 2-element vector
    if length(xcrop) ~= 2 || length(ycrop) ~= 2 || length(zcrop) ~= 2
        error('Crop parameters must be 2-element vectors.');
    end

    % Define the ROI
    roi = [xcrop; ycrop; zcrop];

    % Find points in the specified ROI
    indices = findPointsInROI(pcloud, roi);

    % Select the points from the point cloud
    pcCropped = select(pcloud, indices);
end