function [arrClouds] = readCloudsPlyIntensity(pathname)
%READCLOUDSPLYINTENSITY read directory of .ply files into array, conserve
%intensity value using readPLYIntensity
files = dir(strcat(pathname,'*.ply'));
len = length(files);
fnames=cell(1,len);
arrClouds = repmat(pointCloud(zeros(0,3)),1, len);

for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    fpath=strcat(pathname,filename);
    fnames{i}=fpath;
    arrClouds(i)=readPLYIntensity(strcat(pathname,filename));
end
