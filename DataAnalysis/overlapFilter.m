function [arrFiltered,arrOutliers] = overlapFilter(arrClouds)

    arrFiltered=arrClouds;
    arrOutliers=arrClouds;
    for k=1:8
        pc1=pcdownsample(arrClouds(k),"gridAverage",0.01);
    
        if k==8
            pc2=pcdownsample(arrClouds(1),"gridAverage",0.01);
        else
            pc2=pcdownsample(arrClouds(k+1),"gridAverage",0.01);
        end
        
        if k==1
            pc3=pcdownsample(arrClouds(end),"gridAverage",0.01);
        else
            pc3=pcdownsample(arrClouds(k-1),"gridAverage",0.01);
        end
    
        points1 = pc1.Location;
        points2 = pc2.Location;
        points3 = pc3.Location;
        threshold=0.005;
        % Create KD-Trees for nearest neighbor search
        kdtree2 = KDTreeSearcher(points2);
        kdtree3 = KDTreeSearcher(points3);
    % Initialize logical mask for valid points
        validMask = false(size(points1, 1), 1);
        validLeft=false(size(points1, 1), 1);
        validRight=false(size(points1, 1), 1);
        % Check each point in pcd1 against pcd2 and pcd3
        for i = 1:size(points1, 1)
            point = points1(i, :);
    
            % Find nearest neighbor in pcd2
            [idx2, dist2] = knnsearch(kdtree2, point);
            
            % Find nearest neighbor in pcd3
            [idx3, dist3] = knnsearch(kdtree3, point);
            
            % Check if both distances are within the threshold
           % if dist2 <= threshold && dist3 <= threshold
           if dist2 <= threshold || dist3 <= threshold
                validMask(i) = true;
            end
        end
        % Filter points based on mask
        filteredPoints = pointCloud(points1(validMask, :));
        outlierPoints = pointCloud(points1(~validMask, :));
        arrFiltered(k)=filteredPoints;
        arrOutliers(k)=outlierPoints;
    end
end