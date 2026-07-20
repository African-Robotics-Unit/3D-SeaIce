function shiftedCloud = shiftCloudToSTLMax(cloud, stlPoints)
% SHIFTCLOUDTOSTLMAX Shifts a point cloud so its max Z matches the STL max Z
%
% Inputs:
%   cloud     - pointCloud object or Nx3 matrix of [X Y Z] coordinates
%   stlPoints - Nx3 matrix of STL points (e.g. from stlread().Points / 1000)
%
% Output:
%   shiftedCloud - same type as input, shifted so max(Z) == max(STL Z)

    stlMaxZ = max(stlPoints(:,3));

    if isa(cloud, 'pointCloud')
        pts = double(cloud.Location);
        scanMaxZ = max(pts(:,3));
        pts(:,3) = pts(:,3) - scanMaxZ + stlMaxZ;
        shiftedCloud = pointCloud(pts, ...
            'Color',     cloud.Color, ...
            'Intensity', cloud.Intensity, ...
            'Normal',    cloud.Normal);

    elseif isnumeric(cloud) && size(cloud, 2) == 3
        scanMaxZ = max(cloud(:,3));
        shiftedCloud = cloud;
        shiftedCloud(:,3) = cloud(:,3) - scanMaxZ + stlMaxZ;

    else
        error('Input must be a pointCloud object or an Nx3 numeric matrix.');
    end
end