function [ptCloudRotated, tform] = rotatePlane(ptCloud, model)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
planeNormal = model.Normal;
rotationAxis = cross(planeNormal, [0, 0, 1]);
rotationAngle = acos(dot(planeNormal, [0, 0, 1]) / (norm(planeNormal) * norm([0, 0, 1])));
rotationMatrix = axang2rotm([rotationAxis, rotationAngle]);

%d = model1.Parameters(4);  % This is the 'd' in the plane equation

% The translation vector should be the negative of 'd' along the plane's normal
%trans = -d * planeNormal / norm(planeNormal); 
trans=[0,0,0];
tform = rigidtform3d(rotationMatrix,trans);
ptCloudRotated = pctransform(ptCloud, tform);
end