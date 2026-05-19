
txt = fileread("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\matlab\config\p3_config.json");
params = jsondecode(txt);
pathname = params.preprocessing.pathname;
gridStep = params.preprocessing.gridStep;
x_roi = decodeJsonROI(params.preprocessing.roi.x);
y_roi = decodeJsonROI(params.preprocessing.roi.y);
z_roi = decodeJsonROI(params.preprocessing.roi.z);
nclouds = params.preprocessing.nclouds;
random_seed = params.preprocessing.seed;
startCloud = params.preprocessing.startCloud;
scanOrder = params.preprocessing.scanOrder;

roi=[x_roi,y_roi,z_roi];

[all_clouds, fnames]=rosGetRaw(pathname);
arrLiDAR=cropMergeDownsample(roi,nclouds,gridStep,all_clouds, startCloud);
arrLiDAR_ordered=reorderPointCloud(arrLiDAR,scanOrder);

%Floor transforms
