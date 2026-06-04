function [pcDenoised] = outlierRemoval(pcLiDAR,K,showGraph)
%K=20;
%pcLiDAR=pcCropped;
%nums = 1:pcLiDAR.Count;
pcCropped=pcLiDAR;
 if nargin < 3
        showGraph = false; % Default value
 end
tot = pcLiDAR.Count;
pOutliers = zeros(tot,1);
outliers = zeros(tot,1);
outlierCount = 0;
for i=1:tot
    pt = pcLiDAR.Location(i,:);
    [indices,dists] = findNearestNeighbors(pcLiDAR,pt,K+1);
    dists = dists(2:end);
    sumDist = sum(dists);
    avgDist = sumDist/K;
    maxDist = max(dists);
    term = -1*dists/avgDist;
    %expDists = exp(-1*dists/avgDist);
    expDists = exp(term);
    sumExpDist = sum(expDists);
    localDensity = sumExpDist/K;
    pOutlier = 1-localDensity;
    pOutliers(i)=avgDist;
end
idx = pOutliers<0.05;
idxOutliers = pOutliers >=0.05;

points = pcCropped.Location;
denoisedPoints = points(idx,:);
outlierPts = points(idxOutliers,:);




nums = 1:tot;
sortedArr = sort(pOutliers);
if showGraph
    scatter(nums,sortedArr,15,'filled');
    xIn = denoisedPoints(:,1);
    yIn = denoisedPoints(:,2);
    xOut= outlierPts(:,1);
    yOut= outlierPts(:,2);
    sz=15;
    scatter(xIn,yIn,sz,'blue','filled');
    hold on;
    scatter(xOut,yOut,sz,'red','filled');
    hold off;
 end
pcDenoised = pointCloud(denoisedPoints);
%pcDenoised = pointCloud(denoisedPoints, 'Intensity', denoisedIntensity);
end