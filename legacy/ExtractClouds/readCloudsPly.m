function [arrClouds,fnames, arrRS] = readCloudsPly(pathname)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
files = dir(strcat(pathname,'*.ply'));
len = length(files);
fnames=cell(1,len);
arrClouds = repmat(pointCloud(zeros(0,3)),1, len);
%arrRS=repmat(pointCloud(zeros(0,3)),1, len);
arrRS=cell(1,len);
for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    arrClouds(i)=pcread(strcat(pathname,filename));
    %arrRS(i)=realsense_to_LiDAR(arrClouds(i));
    arrRS{i}=realsense_to_LiDAR(arrClouds(i));
end
