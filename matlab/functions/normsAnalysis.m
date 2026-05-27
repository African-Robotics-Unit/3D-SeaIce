function [sortedPOI,arrIndex,palletIndex,floorIndex,surfIndex] = normsAnalysis(pcRotated, showGraph)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
 if nargin < 2
        showGraph = false; % Default value
 end
[zInc,zDom] = normLevels(pcRotated);

[zIncMax,domMax]=findLocalMax(zDom,zInc);
[zIncMin,domMin]=findLocalMin(zDom,zInc);

topvals=zIncMax(1:5);
topDom=domMax(1:5);

% Calculate absolute differences
differences = abs(topvals - topvals(1));
% Ignore the first element (it's zero)
differences(1) = inf;
% Find the index of the minimum difference
[~, minIndex] = min(differences);
% Extract the closest value
zboard = topvals(minIndex);
zfloor=zIncMax(1);
boardval=domMax(minIndex);
floorval=domMax(1);
topDom([1, minIndex]) = [];
topvals([1, minIndex]) = [];
zSurf=topvals(1);
valSurf=topDom(1)';

if showGraph
    plot(zInc,zDom);
    hold on;
    scatter(zIncMax,domMax, 15, "red","filled")
    scatter(zIncMin,domMin, 15, "green","filled")
    text(zfloor,floorval,"floor",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    text(zboard,boardval,"pallet",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    text(zSurf,valSurf,"surface",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    hold off;
    legend('sum of znormals','max vals','min vals')
end

POI=[zIncMax,zIncMin]; 
sortedPOI=sort(POI);
%finds location of max points in array
palletIndex = find(sortedPOI == zboard);
floorIndex = find(sortedPOI == zfloor);
surfIndex=find(sortedPOI==zSurf);
arrIndex={floorIndex,palletIndex,surfIndex};
end

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

function [sortedZIncrementsForLocalMaxima,sortedLocalMaxima] = findLocalMax(percentDominantZ,zIncrements)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Find the local maxima
localMaxima = islocalmax(percentDominantZ);

% Extract the indices of the local maxima
localMaximaIndices = find(localMaxima);

% Get the z-values and percentages of the local maxima
localMaximaZValues = zIncrements(localMaximaIndices);
localMaximaPercentages = percentDominantZ(localMaximaIndices);


% Sort these values in descending order and get sorting indices
[sortedLocalMaxima, sortIdx] = sort(localMaximaPercentages, 'descend');

% Use these indices to sort the zIncrements values corresponding to local maxima
sortedZIncrementsForLocalMaxima = zIncrements(localMaximaIndices(sortIdx));
%vals=[-0.54,33,0.1,22]
%closeval=0.1

%cutoff=sortedZIncrementsForLocalMaxima(2);
end

function [zvals,minvals] = findLocalMin(percentDominantZ,zIncrements)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

localMin = islocalmin(percentDominantZ);

localMinIndices = find(localMin);

% Get the z-values and percentages of the local maxima
localMinZValues = zIncrements(localMinIndices);
localMinPercentages = percentDominantZ(localMinIndices);

zvals=localMinZValues;
minvals=localMinPercentages;

% Sort these values in descending order and get sorting indices
%[sortedLocalMin, sortIdx] = sort(localMaximaPercentages, 'descend');

% Use these indices to sort the zIncrements values corresponding to local maxima
%sortedZIncrementsForLocalMaxima = zIncrements(localMaximaIndices(sortIdx));
end