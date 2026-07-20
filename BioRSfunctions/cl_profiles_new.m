function [cl,stdcl,mincl,maxcl,acf,acf1D,lags] = cl_profiles_new(X,F)
%
% calculates the average and std dev correlation lengths of a 2-D surface
% of the form F(x,y) in x and y directions
%
% INPUT:
% F = regularly gridded surface model (DEM)
% X = grid points on x-axis
%
% Output:
% cl = correlation length averaged from x and y directions in meters
% stdcl = std dev of cl from profiles in x and y directions
% mincl = min cl from profiles in x and y directions
% maxcl = max cl from profiles in x and y directions
% acf = average autocorrelation function of profiles in x and y directions
% acf1D = list of individual acf's from profiles in x and y directions
% lags = lag distances for the acf
format long
N = length(X); % number of sample points

% autocovariance function calculation
acfx = zeros(N,N); acfy = zeros(N,N);
for i = 1:N
    cx = xcov(F(i,:),'coeff');
    acfx(i,:) = cx(N:2*N-1);
    cy = xcov(F(:,i),'coeff');
    acfy(:,i) = cy(N:2*N-1);
end
acfy = rot90(acfy);

% increase sampling resolution by factor of 10 (no toolbox needed)
factor = 10;
N_int = N * factor;
acfx_int = zeros(N, N_int);
acfy_int = zeros(N, N_int);

x_orig = 1:N;
x_new  = linspace(1, N, N_int);

for i = 1:N
    acfx_int(i,:) = interp1(x_orig, acfx(i,:), x_new, 'spline');
    acfy_int(i,:) = interp1(x_orig, acfy(i,:), x_new, 'spline');
end

acfx = acfx_int; acfy = acfy_int;
acf1D = [acfx; acfy];

% Upsample X the same way
X = interp1(1:N, X, linspace(1, N, N_int), 'spline');

% create lags
N2 = length(X);
lags = linspace(0, X(N2)-X(1), N2);

% correlation lengths calculation for each profile
% in x
clx = zeros(1, N);
for i = 1:N
    k = 1;
    while (acfx(i,k) > 1/exp(1))
        k = k + 1;
    end
    clx(i) = lags(k-1);
end

% in y
cly = zeros(1, N);
for i = 1:N
    k = 1;
    while (acfy(i,k) > 1/exp(1))
        k = k + 1;
    end
    cly(i) = lags(k-1);
end

cl    = (mean(clx) + mean(cly)) / 2;
stdcl = (std(clx)  + std(cly))  / 2;
mincl = min([clx cly]);
maxcl = max([clx cly]);
acf   = (mean(acfx) + mean(acfy)) / 2;
end