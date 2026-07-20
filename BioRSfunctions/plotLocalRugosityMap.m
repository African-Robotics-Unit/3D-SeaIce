function [refRugosityMap, scanRugosityMap, holeMask, refCount, scanCount, xEdges, yEdges] = ...
    plotLocalRugosityMap(refCloud, scanCloud, cellSize, minPtsPerCell)
%PLOTLOCALRUGOSITYMAP Spatially-resolved rugosity map + coverage hole mask.
%
%   plotLocalRugosityMap(refCloud, scanCloud, cellSize)
%   plotLocalRugosityMap(refCloud, scanCloud, cellSize, minPtsPerCell)
%
%   Bins both clouds onto the same XY grid (defined by refCloud's extent,
%   assumed to be full coverage) and computes local rugosity (via
%   computeRugosity.m) within each cell, independently for the reference
%   and scan cloud. Cells with fewer than minPtsPerCell points are left
%   NaN — for the scan cloud, these NaN cells ARE the holes you're
%   looking for; they render as blank patches in the plot.
%
%   INPUTS
%     refCloud, scanCloud   pointCloud objects, same coordinate frame
%     cellSize              grid cell size, same units as point coords
%                            (e.g. metres). Start at ~10-20x point
%                            spacing and refine — too small and every
%                            cell looks like a hole (not enough points to
%                            triangulate), too large and you smooth over
%                            real gaps.
%     minPtsPerCell         (optional) minimum points in a cell to trust
%                            a local rugosity estimate. Default 6.
%
%   OUTPUTS
%     refRugosityMap, scanRugosityMap   2D grids, NaN = insufficient data
%     holeMask                          logical grid — true where ref has
%                                        coverage but scan doesn't
%     refCount, scanCount               2D point-count grids (useful for
%                                        a plain density plot if you want
%                                        to skip the rugosity calc)
%     xEdges, yEdges                    grid bin edges, for reuse
%
%   Requires computeRugosity.m on the path.

if nargin < 4 || isempty(minPtsPerCell)
    minPtsPerCell = 6;
end

refPts  = double(refCloud.Location);
scanPts = double(scanCloud.Location);

% Grid defined by the reference cloud's extent — it's the "full coverage" one
xEdges = min(refPts(:,1)):cellSize:max(refPts(:,1));
yEdges = min(refPts(:,2)):cellSize:max(refPts(:,2));

[refRugosityMap, refCount]   = cellRugosity(refPts,  xEdges, yEdges, minPtsPerCell);
[scanRugosityMap, scanCount] = cellRugosity(scanPts, xEdges, yEdges, minPtsPerCell);

% Hole = reference has adequate coverage, scan doesn't
holeMask = (refCount >= minPtsPerCell) & (scanCount < minPtsPerCell);

%% ── PLOT ──────────────────────────────────────────────────────────────
xCenters = xEdges(1:end-1) + cellSize/2;
yCenters = yEdges(1:end-1) + cellSize/2;

allVals = [refRugosityMap(:); scanRugosityMap(:)];
if all(isnan(allVals))
    error(['No cell in either grid reached minPtsPerCell (%d). cellSize ' ...
           '(%.4g) is likely too small for your point spacing, or ' ...
           'minPtsPerCell is set too high. Check refCount/scanCount ' ...
           '(returned outputs) to see actual points-per-cell before retrying.'], ...
           minPtsPerCell, cellSize);
end
commonClim = [min(allVals, [], 'omitnan'), max(allVals, [], 'omitnan')];
if commonClim(1) == commonClim(2)
    % Only one distinct rugosity value across both grids (e.g. a near-
    % perfectly flat scene, or only one cell cleared the threshold) —
    % pad so caxis doesn't reject a zero-width range.
    commonClim = commonClim + [-0.05, 0.05];
end

figure('Position', [100 100 1400 400]);

subplot(1,3,1);
imagesc(xCenters, yCenters, refRugosityMap, 'AlphaData', ~isnan(refRugosityMap));
set(gca, 'YDir', 'normal', 'Color', [0.85 0.85 0.85]);   % grey = no data
axis equal tight; colorbar; caxis(commonClim);
title('Reference — local rugosity'); xlabel('X'); ylabel('Y');

subplot(1,3,2);
imagesc(xCenters, yCenters, scanRugosityMap, 'AlphaData', ~isnan(scanRugosityMap));
set(gca, 'YDir', 'normal', 'Color', [0.85 0.85 0.85]);
axis equal tight; colorbar; caxis(commonClim);
title('Scan — local rugosity (blank = hole)'); xlabel('X'); ylabel('Y');

subplot(1,3,3);
imagesc(xCenters, yCenters, holeMask);
set(gca, 'YDir', 'normal');
axis equal tight; colormap(gca, [1 1 1; 0.85 0.1 0.1]);   % white = covered, red = hole
refCoveredCells = sum(refCount(:) >= minPtsPerCell);
title(sprintf('Hole mask (%.1f%% of ref-covered area)', 100*sum(holeMask(:))/max(refCoveredCells,1)));
xlabel('X'); ylabel('Y');

end

%% ── LOCAL FUNCTIONS ───────────────────────────────────────────────────────
function [rugosityMap, countMap] = cellRugosity(pts, xEdges, yEdges, minPtsPerCell)
    nX = numel(xEdges) - 1;
    nY = numel(yEdges) - 1;
    rugosityMap = nan(nY, nX);
    countMap    = zeros(nY, nX);

    xIdx = discretize(pts(:,1), xEdges);
    yIdx = discretize(pts(:,2), yEdges);
    valid = ~isnan(xIdx) & ~isnan(yIdx);

    linIdx   = sub2ind([nY, nX], yIdx(valid), xIdx(valid));
    validPts = pts(valid, :);

    % Sort once, then slice contiguous runs — avoids re-scanning every
    % point for every cell (O(N log N) instead of O(nCells * N)).
    [sortedLin, order] = sort(linIdx);
    sortedPts = validPts(order, :);
    groupStarts = [1; find(diff(sortedLin)) + 1; numel(sortedLin) + 1];

    for g = 1:numel(groupStarts) - 1
        rows    = groupStarts(g):groupStarts(g+1) - 1;
        cellLin = sortedLin(rows(1));
        n       = numel(rows);
        countMap(cellLin) = n;
        if n >= minPtsPerCell
            try
                rugosityMap(cellLin) = computeRugosity(sortedPts(rows, :));
            catch
                rugosityMap(cellLin) = NaN;   % degenerate (e.g. collinear) points
            end
        end
    end
end
