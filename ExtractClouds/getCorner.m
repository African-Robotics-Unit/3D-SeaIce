function [ptCloudFloorCropped,minPoint] = getCorner(ptCloudFloor)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[sortedPOI,arrIndex,palletIndex,floorIndex,surfIndex] = normsAnalysis(ptCloudFloor);
ptCloudFloorCropped=cropCloud(ptCloudFloor,[-inf,inf],[-inf,inf],[sortedPOI(floorIndex+1)+0.02,inf]);

points=ptCloudFloorCropped.Location;
% Define Z threshold
Z_threshold = sortedPOI(palletIndex+1); % Replace with your desired Z threshold

% Filter points where Z is below the threshold
filtered_points = points(points(:,3) < Z_threshold, :);

% Further filter points where Y is closest to zero
[~, minYIdx] = min(abs(filtered_points(:,2))); % Find index of Y closest to 0
filtered_points = filtered_points(minYIdx, :); % Select only this point


% [~, minYIdx] = min(filtered_points(:,2)); % Find index of min Y
% filtered_points = filtered_points(minYIdx, :); % Select only this point

% Find the point with the minimum X value
[~, minIdx] = min(filtered_points(:,1)); % Find index of min X
minPoint = filtered_points(minIdx, :);   % Extract the corresponding point
end