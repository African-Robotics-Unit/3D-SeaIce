function [X,Y,F] = linear_interp(x,y,f,scale)

% Linear interpolation of points onto a user defined grid resolution
%
% INPUT:
% x,y,f = 3D coordinates of points
% scale = x-y scale of grid in meters, shouldn't be larger than the spatial
% resolution of x y coordinates in the raw data, default 0.005 m
%
% OUTPUT:
% X = Grid points on x-axis
% Y = Grid points on y-axis
% F = Height values for grid points, where F(X,Y)

xlin=linspace(min(x)+scale,max(x)-scale,((max(x)-min(x))/scale)+2);
ylin=linspace(min(y)+scale,max(y)-scale,((max(y)-min(y))/scale)+2);
[X,Y]=meshgrid(xlin,ylin);
F = TriScatteredInterp(x,y,f,'linear'); % Interpolation
F=F(X,Y);

F(isnan(F))=nanmean(nanmean(F)); 

[N M] = size(F); % force square
dimMax = min(N,M);
F=F(1:dimMax,1:dimMax);
X=X(:,1:dimMax);
Y=Y(1:dimMax,:);

X=X(1,:);
Y=rot90(Y(:,1));

end
