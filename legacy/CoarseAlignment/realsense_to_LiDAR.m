function [pcLiDAR_coords] = realsense_to_LiDAR(pcRealsense_coords)
%converts realsense coordinates to LiDAR coordinates
pcLiDAR_coords_matrix = [pcRealsense_coords.Location(:,3),-pcRealsense_coords.Location(:,1), -1*pcRealsense_coords.Location(:,2)];
pcLiDAR_coords = pointCloud(pcLiDAR_coords_matrix,"Color",pcRealsense_coords.Color);
end