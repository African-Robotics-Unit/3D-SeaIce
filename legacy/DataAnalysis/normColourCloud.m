function [pcColoured] = normColourCloud(maxZCloud)
normsMax=pcnormals(maxZCloud);
absNormals = abs(normsMax);

% Find the dominant direction for each normal
[~, dominantDir] = max(absNormals, [], 2);  % 1=X, 2=Y, 3=Z

% Initialize color matrix
N = size(normsMax, 1);
colours = zeros(N, 3);

% Assign RGB colors based on dominant direction
colours(dominantDir == 1, :) = repmat([1 0 0], sum(dominantDir == 1), 1);  % X -> Red
colours(dominantDir == 2, :) = repmat([0 1 0], sum(dominantDir == 2), 1);  % Y -> Green
colours(dominantDir == 3, :) = repmat([0 0 1], sum(dominantDir == 3), 1);  % Z -> Blue

% Create a new point cloud with colors
pcColoured = pointCloud(maxZCloud.Location, 'Color', uint8(colours * 255));
end