function [pcRealsense_coords] = LiDAR_to_realsense_ros1(pcLiDAR_coords)
%converts LiDAR coordinates to realsense coordinates
pcRealsense_coords_matrix = [pcLiDAR_coords.Location(:,2),pcLiDAR_coords.Location(:,3), -1*pcLiDAR_coords.Location(:,1)];
pcRealsense_coords = pointCloud(pcRealsense_coords_matrix,"Color",pcLiDAR_coords.Color);
end