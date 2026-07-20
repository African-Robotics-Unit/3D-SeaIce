function [cl,stdcl,e,ACFnorm] = cl_3DACF(X,Y,F)

% Calculates the average and standard deviation correlation length of the
% 1/e contour of the three-dimensional autocovariance function of a
% detrended 2-d surface of the form f(x,y)

% Input:
% F = regularly gridded surface model (DEM)
% X = grid points on x-axis
% Y = grid points on y-axis

% Output:
% cl = mean correlation length of 1/e contour through autocovariance
% function
% stdcl = standard deviation of the correlation length
% e = 
% ACFnorm = 

% set up grids
Xsize=size(X,2);
Ysize=size(Y,2);

spacing=X(1,2)-X(1,1);

% calculate normalized 3D ACF and grids
ACF=xcorr2(F);
ACFmax=ACF(Ysize,Xsize);
ACFnorm=1/ACFmax*ACF;

% identify 1/e contour
[C]=contours(ACFnorm,[1/exp(1) 1/exp(1)]);
Csize=size(C,2);

Xabs=Xsize*ones(1,Csize);
Yabs=Ysize*ones(1,Csize);

% contour vectors
vector=sqrt((C(1,:)-Xabs).^2+(C(2,:)-Yabs).^2);

% correlation length
cl=spacing*mean(vector(1,2:Csize));
stdcl=spacing*std(vector(1,2:Csize));
e=sqrt(1-((min(vector(1,2:Csize))^2)/(max(vector(1,2:Csize))^2))); % eccentricity

end