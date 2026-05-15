function [all_clouds, fnames] = rosGetRaw(pathname)
%ROSGETRAW Read LiDAR point cloud messages from ROS bag files.
%
%   Searches for all .bag files in the pathname. For each rosbag, all
%   '/livox/lidar' PointCloud2 messages are read and stored as an array of pointcloud
%   objects. This group of arrays representing the rosbags in the directory
%   is stored in all_clouds.
%
%   Inputs:
%       pathname: String specifying the directory containing the ROS bag files.
%
%   Outputs:
%       all_clouds: Cell array containing the extracted LiDAR message data for each bag file.
%
%       fnames: Cell array of bag file names corresponding to the extracted data.

files = dir(strcat(pathname,'*.bag'));
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
    bags{i}=bagSel;
end

all_clouds=cell(1, len);

for i=1:len
    curLiDAR=LiDARmessages{i};
    nmess=length(curLiDAR);
    clouds=cell(1,nmess);
    for k=1:nmess
        %Read LiDAR message into pointcloud
        myLidar=curLiDAR{k,1};
        xyz_LiDAR = rosReadXYZ(myLidar,"PreserveStructureOnRead",true);
        myfields=rosReadField(myLidar,"intensity");

        % Filter out invalid points
        validPoints = ~(all(xyz_LiDAR == 0, 2) & myfields == 0);
        filteredXYZPoints = xyz_LiDAR(validPoints, :);
        filteredIntensities = myfields(validPoints);
        myLiDARpc=pointCloud(filteredXYZPoints,"Intensity",filteredIntensities);

        clouds{k}=myLiDARpc;
    end
    all_clouds{i}=clouds;
end
end