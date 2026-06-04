function [croppedPtCloud] = cropBrushed(brushedXYZ,ptCloud)
    xMin = min(brushedXYZ(:,1));
    xMax = max(brushedXYZ(:,1));
    
    yMin = min(brushedXYZ(:,2));
    yMax = max(brushedXYZ(:,2));
    
    zMin = min(brushedXYZ(:,3));
    zMax = max(brushedXYZ(:,3));
    
    xyz = ptCloud.Location;
    intensity = ptCloud.Intensity;
    
    % Logical mask for points within the bounding box
    inBox = xyz(:,1) >= xMin & xyz(:,1) <= xMax & ...
            xyz(:,2) >= yMin & xyz(:,2) <= yMax & ...
            xyz(:,3) >= zMin & xyz(:,3) <= zMax;
    
    % Extract filtered data
    croppedXYZ = xyz(inBox, :);
    croppedIntensity = intensity(inBox);
    
    % Create new point cloud
    croppedPtCloud = pointCloud(croppedXYZ, 'Intensity', croppedIntensity);
end