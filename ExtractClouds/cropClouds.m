function [arrCloudsCropped] = cropClouds(arrClouds,roi)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
arrCloudsCropped = repmat(pointCloud(zeros(0,3)), 1, length(arrClouds));
for k=1:length(arrClouds)
   indices=findPointsInROI(arrClouds{k},roi);
   arrCloudsCropped(k) = select(arrClouds{k}, indices);
end
end