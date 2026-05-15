function [processed_pointCloudArray] = cropMergeDownsample(roi,nclouds,DSgridstep,all_clouds)
%CROPMERGEDOWNSAMPLE Converts all_clouds into an array with a single
%pointcloud per viewpoint
%   Input:
%       roi: region of interest for cropping
%       nclouds: number of pointclouds to concatenate
%       DSgridstep: Grid step to use for downsampling
%       all_clouds: Cell array of all rosbag pointclouds
%   Output:
%       processed_pointCloudArray: Array of processed pointclouds
%   Example:
%       z_roi=[-inf,2];
%       y_roi=[-5,5];
%       x_roi=[0,7];
%       roi=[x_roi,y_roi,z_roi];
%
%       arrLiDAR=cropMergeDownsample(roi,50,0.01,all_clouds(2:end));

nviews=length(all_clouds);
croppedClouds=cell(1,nviews);
cur_pointCloudArray = repmat(pointCloud(zeros(0,3)),1, nclouds);

cat_pointCloudArray=repmat(pointCloud(zeros(0,3)),1, nviews);
processed_pointCloudArray=repmat(pointCloud(zeros(0,3)),1, nviews);
sizes_cat=cell(nviews,1);
sizes_proc=cell(nviews,1);
for k=1:nviews
    curView=all_clouds{1,k};
    cur_pointCloudArray = repmat(pointCloud(zeros(0,3)),1, nclouds);
    for i=10:(nclouds+10)
        curCloud=curView{1,i};
        indices = findPointsInROI(curCloud,roi);
        croppedCloud = select(curCloud, indices);
        cur_pointCloudArray(i)=croppedCloud;
    end
    croppedClouds{k}=cur_pointCloudArray;
    cat_pointCloudArray(k)=pccat(cur_pointCloudArray);
    sizes_cat{k}=cat_pointCloudArray(k).Count;
    processed_pointCloudArray(k)=pcdownsample(cat_pointCloudArray(k),"gridAverage",DSgridstep);
    sizes_proc{k}=processed_pointCloudArray(k).Count;
end
end