function [processed_pointCloudArray] = cropMergeDownsample(roi,minsz,DSgridstep,all_clouds)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
nviews=length(all_clouds);
croppedClouds=cell(1,nviews);
cur_pointCloudArray = repmat(pointCloud(zeros(0,3)),1, minsz);

cat_pointCloudArray=repmat(pointCloud(zeros(0,3)),1, nviews);
processed_pointCloudArray=repmat(pointCloud(zeros(0,3)),1, nviews);
sizes_cat=cell(nviews,1);
sizes_proc=cell(nviews,1);
for k=1:nviews
    curView=all_clouds{1,k};
    cur_pointCloudArray = repmat(pointCloud(zeros(0,3)),1, minsz);
    for i=1:minsz
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