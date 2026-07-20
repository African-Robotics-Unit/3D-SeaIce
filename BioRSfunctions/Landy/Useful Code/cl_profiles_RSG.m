function [cl,acf,lags] = cl_profiles_RSG(X,Y,F)
%
% calculates the average correlation lengths of a 2-D surface
% of the form F(x,y) in x and y directions and plots the
% autocovariance function
%
% MODIFIED CODE FROM RSG, CALCULATES TRUE X VERSUS Y CORRELATION LENGTHS
%
% INPUT:
% F = regularly gridded surface model (DEM)
% X = grid points on x-axis
% Y = grid points on y-axis
%
% Output:
% cl = correlation length averaged from x and y directions in meters
% acf = average autocorrelation function of profiles
% lags = lag distances for the acf

format long

N = length(X); % number of sample points

% autocovariance function calculation
acfx = zeros(1,N); acfy = zeros(1,N)';
for i = 1:N
    cx = xcov(F(i,:),'coeff'); % the autocovariance function in x at row i
    acfx = acfx + cx(N:2*N-1); % right-sided version (cumulative sum)
    cy = xcov(F(:,i),'coeff'); % the autocovariance function in y at column i (modified to f(:,1) from f(1,:)
    acfy = acfy + cy(N:2*N-1); % right-sided version (cumulative sum)
end;

acfy=rot90(acfy);

acfx = acfx / N; acfy = acfy / N; % averaging of acfs

% increase sampling resolution by factor of 10
acfx=interp(acfx,10);
acfy=interp(acfy,10);
X=interp(X,10);
Y=interp(Y,10);

N = length(X); % new number of sample points
lags = linspace(0,X(N)-X(1),N);

% correlation lengths calculation
% in x
k = 1;
while (acfx(k) > 1/exp(1))
    k = k + 1;
end;
clx = 1/2*(X(k-1)+X(k)-2*X(1)); % the correlation length in x
% in y
k = 1;
while (acfy(k) > 1/exp(1))
    k = k + 1;
end;
cly = 1/2*(Y(k-1)+Y(k)-2*Y(1)); % the correlation length in y

cl = (clx + cly)/2; % average correlation length in meters

acf = (acfx + acfy)/2; % average acf

end