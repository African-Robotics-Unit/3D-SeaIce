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