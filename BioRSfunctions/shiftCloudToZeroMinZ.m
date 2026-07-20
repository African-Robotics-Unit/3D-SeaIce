function shiftedCloud = shiftCloudToZeroMinZ(cloud)
% SHIFTCLOUDTOZEROMINZ Shifts a point cloud so its minimum Z value is at 0
%
% Input:
%   cloud - pointCloud object or Nx3 matrix of [X Y Z] coordinates
%
% Output:
%   shiftedCloud - same type as input, shifted so min(Z) == 0

    if isa(cloud, 'pointCloud')
        pts = cloud.Location;
        minZ = min(pts(:, 3));
        pts(:, 3) = pts(:, 3) - minZ;
        shiftedCloud = pointCloud(pts, ...
            'Color', cloud.Color, ...
            'Intensity', cloud.Intensity, ...
            'Normal', cloud.Normal);

    elseif isnumeric(cloud) && size(cloud, 2) == 3
        minZ = min(cloud(:, 3));
        shiftedCloud = cloud;
        shiftedCloud(:, 3) = cloud(:, 3) - minZ;

    else
        error('Input must be a pointCloud object or an Nx3 numeric matrix.');
    end
end