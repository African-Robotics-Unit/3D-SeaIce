function ptCloud = meshToPointCloud(vertices, faces, spacing)
% meshToPointCloud - Samples a surface mesh into a point cloud
%
% Syntax:
%   ptCloud = meshToPointCloud(vertices, faces, spacing)
%
% Inputs:
%   vertices - Nx3 array of vertex coordinates
%   faces    - Mx3 array of triangle vertex indices
%   spacing  - approximate spacing between sampled points (in meters)
%
% Outputs:
%   ptCloud  - MATLAB pointCloud object with uniformly sampled points

    sampledPoints = [];

    for i = 1:size(faces,1)
        % Triangle vertices
        v1 = vertices(faces(i,1),:);
        v2 = vertices(faces(i,2),:);
        v3 = vertices(faces(i,3),:);

        % Triangle area
        area = 0.5 * norm(cross(v2 - v1, v3 - v1));

        % Estimate number of points based on triangle area and spacing
        numPts = max(1, ceil(area / (spacing^2)));

        % Sample points using barycentric coordinates
        r1 = sqrt(rand(numPts,1));
        r2 = rand(numPts,1);
        a = 1 - r1;
        b = r1 .* (1 - r2);
        c = r1 .* r2;

        % Compute 3D coordinates of sampled points
        newPts = a * v1 + b * v2 + c * v3;

        sampledPoints = [sampledPoints; newPts];
    end

    % Create point cloud
    ptCloud = pointCloud(sampledPoints);
end
