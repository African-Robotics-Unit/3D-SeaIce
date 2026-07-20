function [cl,stdcl,mincl,maxcl,acf,acf1D,lags] = cl_profiles(X,F)
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
    cx = xcov(F(i,:),'coeff'); % the autocovariance function in x at row i
    acfx(i,:) = cx(N:2*N-1); % right-sided version (cumulative sum)
    cy = xcov(F(:,i),'coeff'); % the autocovariance function in y at column i (modified to f(:,1) from f(1,:)
    acfy(:,i) = cy(N:2*N-1); % right-sided version (cumulative sum)
end;

acfy=rot90(acfy);

% increase sampling resolution by factor of 10
acfx_int=zeros(N,N*10); acfy_int=zeros(N,N*10);
for i = 1:N
    acfx_int(i,:)=interp(acfx(i,:),10);
    acfy_int(i,:)=interp(acfy(i,:),10);
end;
acfx=acfx_int; acfy=acfy_int;
acf1D=[acfx;acfy];
X=interp(X,10);

% create lags
N2 = length(X); % new number of sample points
lags = linspace(0,X(N2)-X(1),N2);

% correlation lengths calculation for each profile
% in x
clx=zeros(1,size(acfx,1));
for i = 1:N
    k = 1;
    while (acfx(i,k) > 1/exp(1))
    k = k + 1;
    end;
    clx(i) = lags(k-1);
end;
% in y
cly=zeros(1,size(acfy,1));
for i = 1:N
    k = 1;
    while (acfy(i,k) > 1/exp(1))
    k = k + 1;
    end;
    cly(i) = lags(k-1);
end;

cl = (mean(clx) + mean(cly))/2; % average correlation length in meters
stdcl = (std(clx) + std(cly))/2; % std dev correlation length in meters
mincl = min([clx cly]);
maxcl = max([clx cly]);


acf = (mean(acfx) + mean(acfy))/2; % average acf

end