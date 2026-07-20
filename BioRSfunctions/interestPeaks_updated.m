function output = interestPeaks_updated(F, X, Y, n, cutoff, overlapTol)
% INTERESTPEAKS_UPDATED  Rayleigh criterion ridge detection on a 2.5D surface grid.
%
%   Scans the surface F(n x m) in both row (X) and column (Y) directions.
%   Applies a modified Rayleigh criterion (Hibler et al. 1972; Wadhams &
%   Horne 1980; Martin 2007; Brenner et al. 2021) with a percentage cutoff.
%   Two peaks are independent only if the intervening trough exceeds
%   cutoff% of the larger peak height.
%
%   Inputs
%     F          : n x m interpolated surface matrix (may contain NaNs)
%     X          : n x m matrix of x-coordinates
%     Y          : n x m matrix of y-coordinates
%     n          : number of rows to scan (X-direction)
%     cutoff     : Rayleigh trough threshold as a percentage (e.g. 25)
%     overlapTol : spatial tolerance (same units as X/Y) for cross-direction
%                  overlap detection. Defaults to 0.05 if not supplied.
%
%   Output
%     output : struct with fields for X- and Y-direction ridge detections
%              and cross-direction overlap candidates (ridge spine points).

if nargin < 6 || isempty(overlapTol)
    overlapTol = 0.05;
end

m = size(F, 2);  % number of columns

% --- X-direction scan (row by row) ---
[xDir] = scanDirection(F, X, Y, n, m, cutoff, 'row');

% --- Y-direction scan (column by column) ---
% Transpose F, X, Y so the same helper can operate column-wise
[yDir] = scanDirection(F', Y', X', m, n, cutoff, 'col');

% --- Cross-direction overlap detection ---
% A point qualifies as a ridge spine candidate if an X-pass peak and a
% Y-pass peak fall within overlapTol in both spatial dimensions.
crossingPeaks = findOverlaps(xDir.peaks, yDir.peaks, overlapTol);

% --- Pack output ---
output.xDir         = xDir;
output.yDir         = yDir;
output.crossingPeaks = crossingPeaks;
output.overlapTol   = overlapTol;
output.cutoff       = cutoff;

end

% =========================================================================
function dir = scanDirection(F, X, Y, nLines, mCols, cutoff, label)
% SCANDIRECTION  Core Rayleigh scan along one axis.
%
%   Operates on rows of F. To scan columns, transpose F/X/Y before calling.
%   Returns a struct with primary peaks, troughs, secondary peaks, gradients
%   and sail angles for each detected independent ridge event.

peaks       = [];   % [x, y, z]  primary (dominant) peak per line
troughs     = [];   % [x, y, z]  qualifying trough per line
peak2       = [];   % [x, y, z]  secondary peak (largest peak <= trough idx)

allLocalMax = [];   % [x, y, z]  every local maximum across all lines

for i = 1:nLines
    row      = F(i, :);
    validIdx = ~isnan(row);
    if sum(validIdx) < 3
        continue  % insufficient points
    end

    origIdx = find(validIdx);
    z       = row(validIdx);

    % --- Local extrema ---
    locMaxMask = islocalmax(z);
    locMinMask = islocalmin(z);
    idxMax     = find(locMaxMask);
    idxMin     = find(locMinMask);

    if isempty(idxMax)
        continue
    end

    % Sort maxima descending by height
    [zMaxSorted, sortOrder] = sort(z(idxMax), 'descend');
    idxMaxSorted = idxMax(sortOrder);

    % Accumulate all local maxima
    for m = 1:numel(idxMaxSorted)
        allLocalMax(end+1, :) = [ ...
            X(i, origIdx(idxMaxSorted(m))), ...
            Y(i, origIdx(idxMaxSorted(m))), ...
            zMaxSorted(m)];
    end

    topIdx = idxMaxSorted(1);  % index of dominant peak in compressed z

    % --- Rayleigh criterion search (leftward from dominant peak) ---
    found    = false;
    idxFound = NaN;
    for k = topIdx:-1:1
        if ismember(k, idxMin)
            diffPct = (zMaxSorted(1) - z(k)) / zMaxSorted(1) * 100;
            if diffPct >= cutoff
                found    = true;
                idxFound = k;
                break
            end
        end
    end

    if ~found
        continue
    end

    % Primary peak
    peakXYZ = [X(i, origIdx(topIdx)), Y(i, origIdx(topIdx)), zMaxSorted(1)];

    % Qualifying trough
    troughXYZ = [X(i, origIdx(idxFound)), Y(i, origIdx(idxFound)), z(idxFound)];

    % Secondary peak: highest local max at or before the trough
    validMaxBefore = idxMax(idxMax <= idxFound);
    if ~isempty(validMaxBefore)
        [~, secIdx] = max(z(validMaxBefore));
        secOrigIdx  = validMaxBefore(secIdx);
        peak2XYZ    = [X(i, origIdx(secOrigIdx)), Y(i, origIdx(secOrigIdx)), z(secOrigIdx)];
    else
        peak2XYZ = [NaN, NaN, NaN];
    end

    peaks(end+1, :)  = peakXYZ;
    troughs(end+1,:) = troughXYZ;
    peak2(end+1, :)  = peak2XYZ;
end

% --- Sail angle calculation (peak-to-trough slope in scan direction) ---
nEvents = size(peaks, 1);
gradients = NaN(nEvents, 1);
angles    = NaN(nEvents, 1);

for j = 1:nEvents
    dZ = peaks(j,3) - troughs(j,3);
    dX = peaks(j,1) - troughs(j,1);
    if dX ~= 0
        gradients(j) = dZ / dX;
        angles(j)    = rad2deg(atan(gradients(j)));
    end
end

% --- Pack direction output ---
dir.label      = label;
dir.peaks      = peaks;
dir.troughs    = troughs;
dir.peak2      = peak2;
dir.allLocalMax = allLocalMax;
dir.gradients  = gradients;
dir.angles     = angles;
dir.avgAngle   = mean(angles, 'omitnan');

end

% =========================================================================
function crossing = findOverlaps(xPeaks, yPeaks, tol)
% FINDOVERLAPS  Identifies spatial coincidences between X- and Y-pass peaks.
%
%   A crossing point is where an X-row peak and a Y-column peak fall within
%   tol in both X and Y simultaneously -- consistent with ridge spine
%   identification in 3D (Muchow & Polojärvi 2024).
%
%   Returns an N x 3 matrix [x, y, z_mean] of crossing candidates.

crossing = [];

if isempty(xPeaks) || isempty(yPeaks)
    return
end

for i = 1:size(xPeaks, 1)
    for j = 1:size(yPeaks, 1)
        dx = abs(xPeaks(i,1) - yPeaks(j,1));
        dy = abs(xPeaks(i,2) - yPeaks(j,2));
        if dx <= tol && dy <= tol
            zMean = mean([xPeaks(i,3), yPeaks(j,3)]);
            crossing(end+1, :) = [ ...
                mean([xPeaks(i,1), yPeaks(j,1)]), ...
                mean([xPeaks(i,2), yPeaks(j,2)]), ...
                zMean];
        end
    end
end

end