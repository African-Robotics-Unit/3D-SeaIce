function [Pf,f1] = raPsd2d(img,extent)
% function raPsd2d(img,res)
%
% Computes and plots radially averaged power spectral density (power
% spectrum) of image IMG with spatial resolution RES.
%
% (C) E. Ruzanski, RCG, 2009

% CHANGED RES TO EXTENT TO SCALE MAGNITUDE OF FREQ TO TRUE VALUES

%% Process image size information
[N M] = size(img);

%% Compute power spectrum
imgf = fftshift(fft2(img));
imgfp = (abs(imgf)/(N*M)).^2;                                               % Normalize

%% Adjust PSD size
dimDiff = abs(N-M);
dimMax = max(N,M);
% Make square
if N > M                                                                    % More rows than columns
    if ~mod(dimDiff,2)                                                      % Even difference
        imgfp = [NaN(N,dimDiff/2) imgfp NaN(N,dimDiff/2)];                  % Pad columns to match dimensions
    else                                                                    % Odd difference
        imgfp = [NaN(N,floor(dimDiff/2)) imgfp NaN(N,floor(dimDiff/2)+1)];
    end
elseif N < M                                                                % More columns than rows
    if ~mod(dimDiff,2)                                                      % Even difference
        imgfp = [NaN(dimDiff/2,M); imgfp; NaN(dimDiff/2,M)];                % Pad rows to match dimensions
    else
        imgfp = [NaN(floor(dimDiff/2),M); imgfp; NaN(floor(dimDiff/2)+1,M)];% Pad rows to match dimensions
    end
end

halfDim = floor(dimMax/2) + 1;                                              % Only consider one half of spectrum (due to symmetry)

%% Compute radially average power spectrum
[X Y] = meshgrid(-dimMax/2:dimMax/2-1, -dimMax/2:dimMax/2-1);               % Make Cartesian grid
[theta rho] = cart2pol(X,Y);                                               % Convert to polar coordinate axes
rho = round(rho);
i = cell(floor(dimMax/2) + 1, 1);
for r = 0:floor(dimMax/2)
    i{r + 1} = find(rho == r);
end
Pf = zeros(1, floor(dimMax/2)+1);
for r = 0:floor(dimMax/2)
    Pf(1, r + 1) = nanmean( imgfp( i{r+1} ) );
end

Pf=10*log10(Pf);                                                            % convert to dB

%% Setup plot
%maxX = 10^(ceil(log10(halfDim)));
%f1 = linspace(1,maxX,length(Pf));                                           % Set abscissa
maxf=dimMax/extent;
f1= linspace(1/extent,maxf,length(Pf));

% Find axes boundaries
%xMin = 0;                                                                   % No negative image dimension
%xMax = ceil(log10(halfDim));
%xRange = (xMin:xMax);
%yMin = floor(log10(min(Pf)));
%yMax = ceil(log10(max(Pf)));
%yRange = (yMin:yMax);

% Create plot axis labels
%xCell = cell(1:length(xRange));
%for i = 1:length(xRange)
    %xRangeS = num2str(10^(xRange(i))*res);
    %xCell(i) = cellstr(xRangeS);
%end
%yCell = num2cell(1:length(yRange));
%for i = 1:length(yRange)
    %yRangeS = num2str(yRange(i));
    %yCell(i) = strcat('10e',cellstr(yRangeS));
%end

%% Generate plot
figure
semilogx(1./f1,Pf)
%loglog(1./f1,Pf)
%plot(1./f1,Pf)
%xlim([xMin xMax]);
%ylim([yMin yMax]);
%set(gca,'YTickLabel',yCell,'YMinorTick','off','XTickLabel',xCell);
xlabel('Normalized Radial Wavelength (m)');
ylabel('Power (dB)');
title('Radially averaged power spectrum')