function [pcCleaned] = getBlockSurface(ptCloud)
addpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\ExtractClouds\");
[sortedPOI,arrIndex,palletIndex,floorIndex,surfIndex] = normsAnalysis(ptCloud);
pcCropped=cropCloud(ptCloud,[0,1],[-inf,inf],[sortedPOI(floorIndex), inf]);
alignFloor(pcCropped);
pcCleaned=pcCropped;
end