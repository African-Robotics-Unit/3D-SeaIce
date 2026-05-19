function writePLYIntensity(filename, ptCloudIn)
    rgb = repmat(ptCloudIn.Intensity, 1, 3);
    ptCloudOut = pointCloud(ptCloudIn.Location, "Color", rgb);   
    pcwrite(ptCloudOut, filename);
end