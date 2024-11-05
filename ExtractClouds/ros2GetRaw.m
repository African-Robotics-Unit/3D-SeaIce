function [all_cloudsLiDAR, all_cloudsRS, fnames] =ros2GetRaw(pathname)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%pathname="C:\Users\agori\Documents\MATLAB\MSc\appDesign\ExtractPC\PancakeALiDAR\";

files = dir(strcat(pathname, 'rosbag2*'));
files = files([files.isdir]); % Ensure only directories

%files = dir(strcat(pathname,'*.bag'));
%names= files.name;
names=files;
len = length(files);
bags = cell(1, len);
LiDARmessages = cell(1, len);
RSmessages= cell(1, len);
all_cloudsRS=cell(1, len);

fnames=cell(1,len);
for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    bagReader = ros2bagreader(strcat(pathname,filename));
    bagSel = select(bagReader,"Topic","/livox/lidar");
    %LiDARmessages{i}=readMessages(bagSel,1:bagSel.NumMessages,'DataFormat','struct');
    LiDARmessages{i}=readMessages(bagSel);
    bags{i}=bagSel;

    if (nargout ==3)
    	bagSelRS=select(bagReader,"Topic","/camera/depth/color/points");
        RSmessages{i}=readMessages(bagSelRS);
        curRS=RSmessages{i};
        nmess=length(curRS);
        cloudsRS=cell(1,nmess);
        for k=1:nmess
            myRS=curRS{k,1};
            xyz_RS = rosReadXYZ(myRS);
            rgb_RS = rosReadRGB(myRS);
            myRSpc =pointCloud(xyz_RS,"Color",rgb_RS);
            cloudsRS{k}=myRSpc;
        end
        all_cloudsRS{i}=cloudsRS;
    end
end

all_cloudsLiDAR=cell(1, len);

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
    all_cloudsLiDAR{i}=clouds;
end

%all_cloudsRS=[];
end