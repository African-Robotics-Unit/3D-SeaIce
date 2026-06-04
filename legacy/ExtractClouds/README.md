# Extract Point Clouds

The purpose of this step is to extract the data from a variety of collection methods to output two arrays containing point cloud data from Realsense sources and LiDAR sources.

| First Header  | Second Header |
| ------------- | ------------- |
| LiDAR (ROS1)              | rosGetRaw        |
| LiDAR & Realsense (ROS2)  | ros2GetRaw       |
| .PLY Realsense Extraction | readCloudsPly    |
| PCD extraction            | preprocess_cloud |

## Ouput:
-	arrRS = array of point clouds from Realsense
-	arrLiDAR = array of point clouds from Livox Avia 

'MATLAB'	'9.13'
'Computer Vision Toolbox'	'10.3'
'ROS Toolbox'	'1.6'

## ROS2 Extra Steps
CMake, VisualStudio Builder Tools, Python 3.9
Need to add Custom message format for using ROS2