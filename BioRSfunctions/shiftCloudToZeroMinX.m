function shiftedCloud = shiftCloudToZeroMinX(cloud)
% SHIFTCLOUDTOZEROMINX Shifts a point cloud so its minimum X value is at 0
%
% Input:
%   cloud - pointCloud object or Nx3 matrix of [X Y Z] coordinates
%
% Output:
%   shiftedCloud - same type as input, shifted so min(X) == 0

    if isa(cloud, 'pointCloud')
        pts = double(cloud.Location);
        minX = min(pts(:,1));
        pts(:,1) = pts(:,1) - minX;
        shiftedCloud = pointCloud(pts, ...
            'Color',     cloud.Color, ...
            'Intensity', cloud.Intensity, ...
            'Normal',    cloud.Normal);

    elseif isnumeric(cloud) && size(cloud, 2) == 3
        minX = min(cloud(:,1));
        shiftedCloud = cloud;
        shiftedCloud(:,1) = cloud(:,1) - minX;

    else
        error('Input must be a pointCloud object or an Nx3 numeric matrix.');
    end
end