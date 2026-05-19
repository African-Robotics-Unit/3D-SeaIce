function ptCloudOut = readPLYIntensity(filename)
    ptCloud = pcread(filename);
    ptCloudOut = pointCloud(ptCloud.Location, "Intensity", ptCloud.Color(:,1));
end