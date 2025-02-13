function [pcRotated] = alignFloor(ptCloud)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
maxDistance=0.02;
MaxNumTrials=3000;
referenceVector=[0, 0, 1];
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,MaxNumTrials);
[pcRotated, tform] = rotatePlane(ptCloud, model1);
end