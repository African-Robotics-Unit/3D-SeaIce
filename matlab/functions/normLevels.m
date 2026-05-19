function [zIncrements,percentDominantZ] = normLevels(pcloud,spacing)
% NORMLEVELS Analyze the distribution of normal vectors in a point cloud.
%
%   [zIncrements, percentDominantZ] = NORMLEVELS(pcloud) analyzes the point 
%   cloud 'pcloud' and returns two arrays: 'zIncrements' and 'percentDominantZ'. 
%   The function divides the z-dimension of the point cloud into increments 
%   (default spacing is 0.02 units) and calculates the sum of the z-components 
%   of the absolute normal vectors within each increment.
%
%   [zIncrements, percentDominantZ] = NORMLEVELS(pcloud, spacing) allows 
%   specifying the 'spacing' between each z-increment. If 'spacing' is not 
%   provided or is empty, the default value of 0.02 is used.
%
%   Inputs:
%       pcloud - A point cloud object
%       spacing - (Optional) A scalar specifying the spacing between z-increments.
%                 Default value is 0.02.
%
%   Outputs:
%       zIncrements - An array of z-values representing the lower bound of each 
%                     z-increment in the point cloud.
%       percentDominantZ - An array of the same length as 'zIncrements'. Each 
%                          element represents the sum of the z-components of the 
%                          absolute normal vectors within the corresponding z-increment.

    if nargin < 2 || isempty(spacing)
        spacing = 0.02; % Default value
    end
zvals=pcloud.Location(:,3);
normsTotal=pcnormals(pcloud);
absNormsTotal=abs(normsTotal);
znorms=absNormsTotal(:,3);
[maxValue, dominantIndexTotal] = max(absNormsTotal, [], 2);
% Find the range of z-values
minZ = min(zvals);
maxZ = max(zvals);

% Generate z-value increments
zIncrements = minZ:spacing:maxZ;

% Initialize array to store percentages
percentDominantZ = zeros(length(zIncrements), 1);
nums=zeros(length(zIncrements), 1);
% Calculate percentages for each increment
for i = 1:length(zIncrements)
    % Determine the range for the current increment
    zLower = zIncrements(i);
    zUpper = zLower + spacing;
    
    % Find points within the current z-range
    inRangeIdx = zvals >= zLower & zvals < zUpper;
    % Calculate the percentage where dominant index is 3
    if any(inRangeIdx)
        %percentDominantZ(i) = 100 * sum(dominantIndexTotal(inRangeIdx) == 3) / sum(inRangeIdx);
        %percentDominantZ(i) =sum(dominantIndexTotal(inRangeIdx) == 3);
        percentDominantZ(i) =sum(znorms(inRangeIdx));
    else
        percentDominantZ(i) = 0;
    end
end
end