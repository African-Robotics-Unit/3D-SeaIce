function pcDenoised = outlierRemoval(pcLiDAR, K, showGraph)
% OUTLIERREMOVAL  Remove outliers based on average K-NN distance.
%   Points whose average distance to K nearest neighbours >= 0.05 are removed.
%
%   Inputs:
%     pcLiDAR   - input point cloud
%     K         - number of nearest neighbours
%     showGraph - (optional) display inlier/outlier scatter plots

if nargin < 3
    showGraph = false;
end

tot      = pcLiDAR.Count;
avgDists = zeros(tot, 1);

for i = 1:tot
    pt            = pcLiDAR.Location(i, :);
    [~, dists]    = findNearestNeighbors(pcLiDAR, pt, K + 1);
    dists         = dists(2:end);           % remove self (distance = 0)
    avgDists(i)   = mean(dists);
end

inlierMask  = avgDists <  0.05;
outlierMask = avgDists >= 0.05;

points        = pcLiDAR.Location;
denoisedPts   = points(inlierMask,  :);
outlierPts    = points(outlierMask, :);

if showGraph
    figure;

    subplot(1, 2, 1);
    scatter(1:tot, sort(avgDists), 15, 'filled');
    yline(0.05, 'r--', 'Threshold = 0.05');
    xlabel('Point index (sorted)');
    ylabel('Avg KNN distance');
    title('Sorted average KNN distances');

    subplot(1, 2, 2);
    scatter(denoisedPts(:,1), denoisedPts(:,2), 15, 'blue', 'filled');
    hold on;
    scatter(outlierPts(:,1),  outlierPts(:,2),  15, 'red',  'filled');
    hold off;
    xlabel('X'); ylabel('Y');
    axis equal;
    legend('Inliers', 'Outliers');
    title('Inliers vs Outliers (XY)');
end

pcDenoised = pointCloud(denoisedPts);
end