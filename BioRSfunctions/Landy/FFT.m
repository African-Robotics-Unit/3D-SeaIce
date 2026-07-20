function [R] = FFT(F,scale,cutoff)

% Detrends a regularly gridded, square, two-dimensional surface F using
% the fast fourier transform and filtering below a cutoff wavelength.

% Input:
% F = regularly gridded surface model (DEM)
% scale = grid spacing in meters
% cutoff = length in meters above which surface wavelengths are removed from the
% surface model, default = 0.1 m (i.e. > largest cl measured over snow or
% ice)

% Output:
% R = gridded surface model reconstructed from wavelengths above the cutoff

% Usage:
% [R] = FFT(F,0.002,0.05)
% [R] = FFT(F,0.001,[])

% Take 2D fast fourier transform
ft=fft2(F);

% Identify frequencies in units of 'cycles across surface'
nx=size(ft,2);
ny=size(ft,1);
%cxrange=[0:nx/2,-nx/2+1:-1]; %old
%cyrange=[0:ny/2,-ny/2+1:-1];
cxrange=[0:nx/2,-nx/2:-1];
cyrange=[0:ny/2,-ny/2:-1];
[cx,cy]=meshgrid(cxrange,cyrange);
% Identify cutoff frequency
if isempty(cutoff)
    cutoff=0.1;
end;

% Filter only frequencies lower than cutoff and reconstruct
%cyclex=(nx*scale)/cutoff; %old
%cycley=(ny*scale)/cutoff;
cyclex=((nx-1)*scale)/cutoff;
cycley=((ny-1)*scale)/cutoff;
%keep=abs(cx)<cyclex & abs(cy)<cycley;
keep=abs(cx)>cyclex & abs(cy)>cycley;
partft=ft.*keep;
R=real(ifft2(partft));

end