function [minsz, testout] = getMinSz(all_clouds,fnames)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
nviews=length(all_clouds);
sizes=cell(nviews,1);
arrTest={};
pointCloudArray = repmat(pointCloud(zeros(0,3)), size(all_clouds));
for k=1:nviews
    sizes{k}=length(all_clouds{k});
    cur=all_clouds{1,k};
    arrTest{k}=cur{1,3};
    pointCloudArray(k)=cur{1,3};
end
% Assuming fnames and all_clouds are your cell arrays
combinedData = [fnames', sizes];

% Convert the combined cell array to a table
combinedTable = cell2table(combinedData);

% Optionally, you can provide column names for better readability
combinedTable.Properties.VariableNames = {'Filenames', 'NClouds'};

% Display the combined table
disp(combinedTable);
minsz=min(cell2mat(sizes));
disp("Min size=" + int2str(minsz));
testout=pccat(pointCloudArray);
end