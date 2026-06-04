function [pcTop] = extractTopCloud(maxZCloud)
% Step 1: Identify Z-dominant points
normsMax=pcnormals(maxZCloud);
absNormals = abs(normsMax);
[~, dominantDir] = max(absNormals, [], 2);  % 1=X, 2=Y, 3=Z

% Step 2: Filter out Z-dominant points (keep only X or Y dominant)
topIndices = dominantDir == 3;

% Get the corresponding points and normals
topPoints = maxZCloud.Location(topIndices, :);
%topNormals = normsMax(topIndices, :);

% Step 5: Create new point cloud and display
pcTop= pointCloud(topPoints);
end