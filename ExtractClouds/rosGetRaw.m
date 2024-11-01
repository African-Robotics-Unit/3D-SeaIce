function [all_clouds, fnames] =rosGetRaw(pathname)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%pathname="C:\Users\agori\Documents\MATLAB\MSc\appDesign\ExtractPC\PancakeALiDAR\";
files = dir(strcat(pathname,'*.bag'));
names= files.name;
len = length(files);
bags = cell(1, len);
LiDARmessages = cell(1, len);
fnames=cell(1,len);
for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    bagReader = rosbagreader(strcat(pathname,filename));
    bagSel = select(bagReader,"Topic","/livox/lidar");
    LiDARmessages{i}=readMessages(bagSel,1:bagSel.NumMessages,'DataFormat','struct');
    %LiDARmessages{i}=readMessages(bagSel);
    bags{i}=bagSel;
end

all_clouds=cell(1, len);

for i=1:len
    curLiDAR=LiDARmessages{i};
    %curLiDAR=bags{i};
    nmess=length(curLiDAR);
    %nmess=curLiDAR.NumMessages;
    clouds=cell(1,nmess);
    for k=1:nmess
        myLidar=curLiDAR{k,1};
        %myLidar=readMessages(curLiDAR);
        xyz_LiDAR = rosReadXYZ(myLidar,"PreserveStructureOnRead",true);
        myfields=rosReadField(myLidar,"intensity");
        validPoints = ~(all(xyz_LiDAR == 0, 2) & myfields == 0);

        % Filter out invalid points
        filteredXYZPoints = xyz_LiDAR(validPoints, :);
        filteredIntensities = myfields(validPoints);

        %myLiDARpc=pointCloud(xyz_LiDAR,"Intensity",myfields);
        myLiDARpc=pointCloud(filteredXYZPoints,"Intensity",filteredIntensities);
        clouds{k}=myLiDARpc;
    end
    all_clouds{i}=clouds;
end
end