function [sortedPOI,p1Index,p2Index,p3Index] = zNormsAnalysis(pcRotated, showGraph)
 if nargin < 2
        showGraph = false; % Default value
 end
[zInc,zDom] = normLevels(pcRotated);

[zIncMax,domMax]=findLocalMax(zDom,zInc);
[zIncMin,domMin]=findLocalMin(zDom,zInc);

topvals=zIncMax(1:3);
topDom=domMax(1:3);


if showGraph
    plot(zInc,zDom);
    hold on;
    scatter(zIncMax,domMax, 15, "red","filled")
    scatter(zIncMin,domMin, 15, "green","filled")
    text(topvals(1),topDom(1),"p1",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    text(topvals(2),topDom(2),"p2",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    text(topvals(3),topDom(3),"p3",'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    hold off;
    legend('sum of znormals','max vals','min vals')
    xlabel("Z (m)")
    ylabel("Sum of Z normals")
end

POI=[zIncMax,zIncMin]; 
sortedPOI=sort(POI);

p1Index=find(sortedPOI == topvals(1));
p2Index=find(sortedPOI == topvals(2));
p3Index=find(sortedPOI == topvals(3));

end