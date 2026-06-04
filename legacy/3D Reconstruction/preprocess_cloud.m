function [dataset,ptCloudMerge_crop,ptCloudDownsample_crop,ptCloudMerge, ptCloudDownsample] = preprocess_cloud(pathname)
dataset = struct('Cloud',{}, 'filename', {},'croppedCloud', {});
files = dir(strcat(pathname,'*.pcd'));
len = length(files);

%Manually determined parameters that only show the pancake in frame
xmax=5;
zmax=2;

%Populating the dataset array
for i=1:len
    dataset(i).filename = files(i).name;
    dataset(i).Cloud = pcread(strcat(pathname,files(i).name));
    cloud = dataset(i).Cloud;
    roi = [0,xmax;-inf,inf;0,zmax];
    indices = findPointsInROI(cloud,roi);
    dataset(i).croppedCloud = select(dataset(i).Cloud, indices);
end

%Constructiong ptCloudMerge, a single point cloud combining all Cloud fields in the dataset
ptCloudMerge=dataset(1).Cloud;
%Constructiong ptCloudMerge_crop, a single point cloud combining all croppedCloud fields in the dataset
ptCloudMerge_crop = dataset(1).croppedCloud;

%Populating ptCloudMerge and ptCloudMerge_crop
for i=2:len
    ptCloudMerge_crop = pcmerge(ptCloudMerge_crop, dataset(i).croppedCloud,0.001);
    ptCloudMerge = pcmerge(ptCloudMerge, dataset(i).Cloud,0.001);
end

%Downsampling large ptCloudMerge and ptCloudMerge_crop into 0.05 grid steps
gridStep = 0.05;
ptCloudDownsample_crop = pcdownsample(ptCloudMerge_crop,'gridAverage',gridStep);
ptCloudDownsample = pcdownsample(ptCloudMerge,'gridAverage',gridStep);
end