function [arrFiltered, arrOutliers] = overlapFilterOptimal(arrClouds)

    nClouds = numel(arrClouds);
    arrFiltered = arrClouds;
    arrOutliers = arrClouds;

    gridStep = 0.01;
    threshold = 0.005;

    arrDown = arrClouds;
    for k = 1:nClouds
        arrDown(k) = pcdownsample(arrClouds(k), "gridAverage", gridStep);
    end

    for k = 1:nClouds

        pc1 = arrDown(k);

        if k == nClouds
            pc2 = arrDown(1);
        else
            pc2 = arrDown(k+1);
        end

        if k == 1
            pc3 = arrDown(nClouds);
        else
            pc3 = arrDown(k-1);
        end

        points1 = pc1.Location;
        points2 = pc2.Location;
        points3 = pc3.Location;


        % Build KD-trees
        kdtree2 = KDTreeSearcher(points2);
        kdtree3 = KDTreeSearcher(points3);

        % Vectorised nearest-neighbour search
        [~, dist2] = knnsearch(kdtree2, points1);
        [~, dist3] = knnsearch(kdtree3, points1);

        % Keep points that overlap with either neighbour
        validMask = dist2 <= threshold | dist3 <= threshold;

        intensity1 = pc1.Intensity;

        arrFiltered(k) = pointCloud(points1(validMask, :), ...
            "Intensity", intensity1(validMask));

        arrOutliers(k) = pointCloud(points1(~validMask, :), ...
            "Intensity", intensity1(~validMask));
    end
end