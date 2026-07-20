function [cl,stdcl,e,ACFnorm] = cl_3DACF_new(X,Y,F)
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
% e = eccentricity of the 1/e contour
% ACFnorm = normalised 3D autocovariance function

% set up grids
Xsize = size(X,2);
Ysize = size(Y,2);
spacing = X(1,2) - X(1,1);

% calculate normalised 3D ACF using FFT (replaces xcorr2)
% xcorr2(F) is equivalent to a full 2D cross-correlation of F with itself
[m, n] = size(F);
% pad to avoid circular wrap-around (same output size as xcorr2)
M = 2*m - 1;
N = 2*n - 1;
F_fft = fft2(F, M, N);
ACF = ifft2(F_fft .* conj(F_fft));
ACF = real(ACF);                          % discard negligible imaginary rounding
% xcorr2 returns a (2m-1) x (2n-1) matrix; rearrange quadrants to match
ACF = circshift(ACF, [m-1, n-1]);

ACFmax = ACF(Ysize, Xsize);
ACFnorm = (1/ACFmax) * ACF;

% identify 1/e contour (contours is base MATLAB)
C = contours(ACFnorm, [1/exp(1), 1/exp(1)]);
Csize = size(C, 2);
Xabs = Xsize * ones(1, Csize);
Yabs = Ysize * ones(1, Csize);

% contour vectors
vector = sqrt((C(1,:) - Xabs).^2 + (C(2,:) - Yabs).^2);

% correlation length
cl    = spacing * mean(vector(1, 2:Csize));
stdcl = spacing * std(vector(1, 2:Csize));
e     = sqrt(1 - (min(vector(1,2:Csize))^2) / (max(vector(1,2:Csize))^2));
end