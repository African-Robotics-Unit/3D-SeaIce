# 3D Reconstruction of Pancake Sea Ice using LiDAR and Cameras

MSc student: Agoritsa Spirakis
Supervisor: Robyn Verrinder
Co-supervisor: James Hepworth
# IGARSS 2023

# Pipeline explanation
## ExtractLiDAR.mlx
- Input: ros folders with extracted PCDs
- Applies function preprocess_cloud
  - Combines the multiple PCDs and merges it into 1
  - Downsamples to 2cm using grid method
- Output: Data in point cloud format
## TransformLiDAR.mlx
- Input: Variables created from ExtractLiDAR.mlx
- Performs appropriate cropping
- Gets RS transforms from ALN file (extractALNtransform.m)
- Applies initial transform to LiDAR clouds (apply_RS_tforms.m)
- Output: writes aligned files to PLY
## applyICP.mlx
- Input: PLY files
- Using Glira (2015) implements ICP algorithm
- Output: ICP object
## exportICPclouds.mlx
- Input:ICP object
- Ouput: afterICP aligned files to PLY 
