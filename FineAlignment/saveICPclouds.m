function saveICPclouds(icp, folder,nviews)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
addpath(genpath('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\LiDAR Extraction'));
arrLetters=["A", "B", "C", "D", "E", "F", "G", "H"];

for k=1:nviews
    %arrClouds(k)=pointCloud(icp.loadPC(k);
    fname= strcat(folder,arrLetters{k},"-after.ply");
    icp.exportPC(k,fname);
end
end