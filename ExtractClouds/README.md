# Extract Point Clouds

The purpose of this step is to extract the data from a variety of collection methods to output two arrays containing point cloud data from Realsense sources and LiDAR sources.

Ouput:
-	arrRS = array of point clouds from Realsense
-	arrLiDAR = array of point clouds from Livox Avia 
| First Header  | Second Header |
| ------------- | ------------- |
| LiDAR (ROS1)              | rosGetRaw        |
| LiDAR & Realsense (ROS2)  | ros2GetRaw       |
| .PLY Realsense Extraction | readCloudsPly    |
| PCD extraction            | preprocess_cloud |
