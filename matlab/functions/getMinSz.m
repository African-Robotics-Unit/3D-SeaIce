function [minsz, testout] = getMinSz(all_clouds,fnames)
%GETMINSZ Provides a summary of the number of pointclouds in each bagfile
% and provides the minimum number of frames present in all files (minsz).
% Additionally provides a testout pointcloud with a single frame from each
% viewpoint for cropping parameter purposes.
%
% Inputs:
%       all_clouds: Cell array containing the extracted LiDAR message data
%                    for each bag file.
%
%       fnames: Cell array of bag file names corresponding to the
%                    extracted data.
% Outputs:
%       minsz: Integer representing minimum number of frames
%       testout: Pointcloud comprising of a single frame per viewpoint
%       concatenated.
nviews=length(all_clouds);
sizes=cell(nviews,1);
pointCloudArray = repmat(pointCloud(zeros(0,3)), size(all_clouds));
for k=1:nviews
    sizes{k}=length(all_clouds{k});
    cur=all_clouds{1,k};
    pointCloudArray(k)=cur{1,3};
end

% Table output
combinedData = [fnames', sizes];
combinedTable = cell2table(combinedData);
combinedTable.Properties.VariableNames = {'Filenames', 'NClouds'};

disp(combinedTable);
minsz=min(cell2mat(sizes));
disp("Min size=" + int2str(minsz));
testout=pccat(pointCloudArray);
end