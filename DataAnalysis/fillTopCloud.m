function [pcTopFilled] = fillTopCloud(pcTop, maxZCloud,xBinSize)
% Set bin size along X
%xBinSize = 0.01;

% Extract data
%filteredPoints = pcTopCropFiltered.Location;
normsMax=pcnormals(maxZCloud);
absNormals = abs(normsMax);
[~, dominantDir] = max(absNormals, [], 2);  % 1=X, 2=Y, 3=Z

filteredPoints = pcTop.Location;
allPoints = maxZCloud.Location;
nonTopIndices = dominantDir ~= 3;
nonTopPoints = allPoints(nonTopIndices, :);

% Z filter for discarded points
nonTopPoints = nonTopPoints(nonTopPoints(:,3) > 0.1, :);

% Initialize list of added points
addedPoints = [];

% Define X range from filtered data
xMin = min(filteredPoints(:,1));
xMax = max(filteredPoints(:,1));
xEdges = xMin:xBinSize:xMax;

for i = 1:(length(xEdges)-1)
    xLow = xEdges(i);
    xHigh = xEdges(i+1);

    % Get filtered points in this X slice
    inBinFilt = filteredPoints(:,1) >= xLow & filteredPoints(:,1) < xHigh;
    sliceFiltPoints = filteredPoints(inBinFilt, :);

    if isempty(sliceFiltPoints)
        continue;
    end

    % Determine Y-range in this slice
    yMin = min(sliceFiltPoints(:,2));
    yMax = max(sliceFiltPoints(:,2));

    % Find matching discarded points in same X and Y range
    inX = nonTopPoints(:,1) >= xLow & nonTopPoints(:,1) < xHigh;
    inY = nonTopPoints(:,2) >= yMin & nonTopPoints(:,2) <= yMax;

    inSlice = inX & inY;
    addedPoints = [addedPoints; nonTopPoints(inSlice, :)];  %#ok<AGROW>
end

% Combine and color code
N_filtered = size(filteredPoints, 1);
N_added = size(addedPoints, 1);

colorsFiltered = repmat([0 0 1], N_filtered, 1); % blue
colorsAdded = repmat([1 0 0], N_added, 1);       % red

pcTopFilledPoints = [filteredPoints; addedPoints];
pcTopFilledColors = [colorsFiltered; colorsAdded];

pcTopFilled = pointCloud(pcTopFilledPoints, 'Color', uint8(pcTopFilledColors * 255));
end