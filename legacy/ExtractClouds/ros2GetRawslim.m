function [all_cloudsLiDAR, arrRSraw, fnames] =ros2GetRawslim(pathname)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%pathname="C:\Users\agori\Documents\MATLAB\MSc\appDesign\ExtractPC\PancakeALiDAR\";
files = dir(strcat(pathname, 'rosbag2*'));
files = files([files.isdir]); % Ensure only directories


len = length(files);
%len=1;

arrRSraw=cell(1, len);
fnames=cell(1,len);
all_cloudsLiDAR=cell(1, len);


for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    bagReader = ros2bagreader(strcat(pathname,filename));
    bagSel = select(bagReader,"Topic","/livox/lidar");
    %LiDARmessages{i}=readMessages(bagSel);
    %curLiDAR=LiDARmessages{i};
    curLiDAR=readMessages(bagSel);
    nmess=length(curLiDAR);
    clouds=cell(1,nmess);

    for k=1:nmess
        myLidar=curLiDAR{k,1};

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
    all_cloudsLiDAR{i}=clouds;


    %Reading RS Data
    bagSelRS=select(bagReader,"Topic","/camera/depth/color/points");
    %RSmessages{i}=readMessages(bagSelRS);
    %curRS=RSmessages{i};
    curRS=readMessages(bagSelRS);
    myRS=curRS{4,1};
    xyz_RS = rosReadXYZ(myRS);
    rgb_RS = rosReadRGB(myRS);
    myRSpc =pointCloud(xyz_RS,"Color",rgb_RS);
    arrRSraw{i}=realsense_to_LiDAR_ros2(myRSpc);
end
end