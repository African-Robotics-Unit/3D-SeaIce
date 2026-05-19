function [sortedZIncrementsForLocalMaxima,sortedLocalMaxima] = findLocalMax(percentDominantZ,zIncrements)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Find the local maxima
localMaxima = islocalmax(percentDominantZ);

% Extract the indices of the local maxima
localMaximaIndices = find(localMaxima);

% Get the z-values and percentages of the local maxima
localMaximaZValues = zIncrements(localMaximaIndices);
localMaximaPercentages = percentDominantZ(localMaximaIndices);


% Sort these values in descending order and get sorting indices
[sortedLocalMaxima, sortIdx] = sort(localMaximaPercentages, 'descend');

% Use these indices to sort the zIncrements values corresponding to local maxima
sortedZIncrementsForLocalMaxima = zIncrements(localMaximaIndices(sortIdx));
%vals=[-0.54,33,0.1,22]
%closeval=0.1

%cutoff=sortedZIncrementsForLocalMaxima(2);
end