function [pd,spacing] = points_info(data)
% Info on point cloud in meters
% Input x y points
% Output pd = point density per square meter and spacing = point spacing in
% meters (horizontal resolution)

x=data(:,1);
y=data(:,2);

size_x=max(x)-min(x);
size_y=max(y)-min(y);
area=size_x*size_y;

pd=numel(x)/area;
spacing=1/sqrt(pd);

end

