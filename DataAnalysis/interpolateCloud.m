function [F] = interpolateCloud(pcNew, scale)
x = double(pcNew.Location(:,1));
y = double(pcNew.Location(:,2));
f = double(pcNew.Location(:,3));
%scale = 0.02;


xlin=linspace(min(x)+scale,max(x)-scale,((max(x)-min(x))/scale)+2);
ylin=linspace(min(y)+scale,max(y)-scale,((max(y)-min(y))/scale)+2);
[X,Y]=meshgrid(xlin,ylin);
%F = scatteredInterpolant(x,y,f,'natural'); % Interpolation
F = scatteredInterpolant(x,y,f,'linear'); % Interpolation
F=F(X,Y);
DT = delaunayTriangulation(x,y);
C = convexHull(DT);
% Create a mask
[in, on] = inpolygon(X, Y, DT.Points(C,1), DT.Points(C,2)); % Check if grid points are inside the convex hull
mask = in | on; % Include points on the boundary

% Apply the mask
F(~mask) = NaN; % Set values outside the convex hull to NaN
end
