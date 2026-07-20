function [R,params,acf1D,ACFnorm,lags] = roughness(data,scale,detrend,cutoff,plot)

% Calculates surface roughness statistics for a 2D surface of the form
% F(x,y) generated from raw trivariate data.

% Input:
% data = trivariate data
% scale = horizontal resolution for gridded surface
% detrend = choose detrending algorithm, []=none, 1=OLSR, 2=FFT
% cutoff = choose distance between OLSR plane breaks or cutoff wavelength
% for FFT, []=default cutoff of 0.1 m
% plot = optional plotting, type [1] 'detrend' for before and after two and
% three-dimensional plots of the detrending procedure, [2] 'hdf' for height
% distribution function of detrended rough surface, [3] 'acf' for
% average autocovariance function of detrended rough surface, and [4] 'iso'
% for a plot of the 1/e contour of the 2D ACF and angular variation of the
% 1/e contour, to illustrate isotropy

% Output:
% R = gridded surface model reconstructed after detrending
% params = look-up table containing roughness parameters, including, in order:
    % h = rms height
    % cl1 = average correlation length calculated from profiles across R in x
    % and y dimensions
    % stdcl1 = std dev of correlation lengths from profiles
    % mincl1 = min of cl from profiles
    % maxcl1 = max of cl from profiles
    % cl2 = average correlation length calculated from the 1/e contour of the
    % 2D autocovarince function of R
    % stdcl2 = standard deviation of cl2
    % ecl2 = eccentricity of cl2, i.e. test for isotropy
% acf = 1-D autocorrelation function from profiles
% ACFnorm = 2-D auotcorrelation function from cross-correlation
% lags = lag distances for autocorrelation functions

% Usage:
% [R,params,acf,ACFnorm,lags] = roughness(data,0.002,[],[])
% [R,params,acf,ACFnorm,lags] = roughness(data,0.01,2,0.25,'acf')

% Copyright J.C. Landy 2013

% Trim
%data=crop(data);

% Create Trivariate Data
x=data(:,1); y=data(:,2); f=data(:,3);

% Linear Interpolation
[X,Y,F] = linear_interp(x,y,f,scale);

% Detrend
if isempty(detrend)
    R=F-mean(mean(F));
elseif isnumeric(detrend)
    if detrend==1
        [R] = OLSR(F,scale,cutoff);
    elseif detrend==2
        [R] = FFT(F,scale,cutoff);
    end;
end;

% RMS Height
[h] = rmsh(R);

% Correlation Lengths
[cl1,stdcl1,mincl1,maxcl1,acf,acf1D,lags] = cl_profiles(X,R);
[cl2,stdcl2,ecl2,ACFnorm] = cl_3DACF(X,Y,R);

% Parameter Look-up Table
params=[h cl1 stdcl1 mincl1 maxcl1 cl2 stdcl2 ecl2];

% Display in Command Window
disp('RMS Height'); disp(h);
disp('1D Correlation Length mean std min max'); disp([cl1 stdcl1 mincl1 maxcl1]);
disp('2D Correlation Length mean std ecc'); disp([cl2 stdcl2 ecl2]);

% Optional Plotting
if nargin==5 
    if strcmp(plot,'detrend');
        reconstruction_plot(F,R,scale,cutoff)
    elseif strcmp(plot,'hdf');
        hdf_plot(R,100);
    elseif strcmp(plot,'acf');
        acf_plot(acf,lags,scale,ACFnorm);
    elseif strcmp(plot,'iso');
        iso_plot(R,ACFnorm,scale);
    end;
end; 

end

