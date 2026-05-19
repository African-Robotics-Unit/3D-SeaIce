function [zvals,minvals] = findLocalMin(percentDominantZ,zIncrements)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

localMin = islocalmin(percentDominantZ);

localMinIndices = find(localMin);

% Get the z-values and percentages of the local maxima
localMinZValues = zIncrements(localMinIndices);
localMinPercentages = percentDominantZ(localMinIndices);

zvals=localMinZValues;
minvals=localMinPercentages;

% Sort these values in descending order and get sorting indices
%[sortedLocalMin, sortIdx] = sort(localMaximaPercentages, 'descend');

% Use these indices to sort the zIncrements values corresponding to local maxima
%sortedZIncrementsForLocalMaxima = zIncrements(localMaximaIndices(sortIdx));
end