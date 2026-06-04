function [filteredPtCloud] = filterIntensityCloud(ptCloud,intMin,intMax)

% Extract intensity values
intensity = ptCloud.Intensity;

% Logical mask for intensity range
inRange = intensity >= intMin & intensity <= intMax;

% Extract the corresponding XYZ coordinates
filteredXYZ = ptCloud.Location(inRange, :);

% Extract the corresponding intensity values
filteredIntensity = intensity(inRange);

% Create new point cloud
filteredPtCloud = pointCloud(filteredXYZ, 'Intensity', filteredIntensity);
end