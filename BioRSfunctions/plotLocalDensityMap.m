function [refDensityMap, scanDensityMap, refCount, scanCount, xEdges, yEdges, axRef, axScan] = ...
    plotLocalDensityMap(refCloud, scanCloud, cellSize, overlayPoints)
%PLOTLOCALDENSITYMAP Spatially-resolved point cloud density comparison.
%
%   plotLocalDensityMap(refCloud, scanCloud, cellSize)
%   plotLocalDensityMap(refCloud, scanCloud, cellSize, overlayPoints)
%
%   Bins both clouds onto the same XY grid (defined by refCloud's extent)
%   and computes point density (points per unit area) per cell,
%   independently for reference and scan, for direct visual comparison of
%   sampling coverage across the surface. No triangulation, no
%   minPtsPerCell threshold — an empty cell is density 0, which is
%   already the correct value, so sparse/empty regions show up naturally
%   as the darkest cells rather than needing a separate hole mask.
%
%   INPUTS
%     refCloud, scanCloud   pointCloud objects, same coordinate frame
%     cellSize              grid cell size, same units as point coords
%                            (e.g. metres)
%     overlayPoints         (optional) scatter the raw point cloud on top
%                            of each density map. Default true.
%
%   OUTPUTS
%     refDensityMap, scanDensityMap   2D grids, points per unit area
%                                      (e.g. points/m^2 if cellSize is in
%                                      metres) — same colour scale on both
%                                      for direct comparison
%     refCount, scanCount             2D raw point-count grids
%     xEdges, yEdges                  grid bin edges, for reuse
%     axRef, axScan                   axes handles for further tweaking
%
%   Example:
%     plotLocalDensityMap(refCloud, scanCloud, 0.01);

if nargin < 4 || isempty(overlayPoints)
    overlayPoints = true;
end

refPts  = double(refCloud.Location);
scanPts = double(scanCloud.Location);

xEdges = min(refPts(:,1)):cellSize:max(refPts(:,1));
yEdges = min(refPts(:,2)):cellSize:max(refPts(:,2));

refCount  = cellCount(refPts,  xEdges, yEdges);
scanCount = cellCount(scanPts, xEdges, yEdges);

cellArea       = cellSize^2;
refDensityMap  = refCount  / cellArea;
scanDensityMap = scanCount / cellArea;

%% ── PLOT ──────────────────────────────────────────────────────────────
xCenters = xEdges(1:end-1) + cellSize/2;
yCenters = yEdges(1:end-1) + cellSize/2;

commonClim = [0, max([refDensityMap(:); scanDensityMap(:)])];
if commonClim(2) == 0
    commonClim = [0, 1];   % avoid zero-width caxis if both clouds are empty
end

figure('Position', [100 100 950 420]);

subplot(1,2,1);
imagesc(xCenters, yCenters, refDensityMap);
set(gca, 'YDir', 'normal');
axis equal tight; colorbar; caxis(commonClim);
hold on;
if overlayPoints
    scatter(refPts(:,1), refPts(:,2), 3, 'w', 'filled', 'MarkerFaceAlpha', 0.35);
end
title('Reference — point density'); xlabel('X'); ylabel('Y');
axRef = gca;

subplot(1,2,2);
imagesc(xCenters, yCenters, scanDensityMap);
set(gca, 'YDir', 'normal');
axis equal tight; colorbar; caxis(commonClim);
hold on;
if overlayPoints
    scatter(scanPts(:,1), scanPts(:,2), 3, 'w', 'filled', 'MarkerFaceAlpha', 0.35);
end
title('Scan — point density'); xlabel('X'); ylabel('Y');
axScan = gca;

end

%% ── LOCAL FUNCTIONS ───────────────────────────────────────────────────────
function countMap = cellCount(pts, xEdges, yEdges)
    nX = numel(xEdges) - 1;
    nY = numel(yEdges) - 1;

    xIdx = discretize(pts(:,1), xEdges);
    yIdx = discretize(pts(:,2), yEdges);
    valid = ~isnan(xIdx) & ~isnan(yIdx);

    linIdx   = sub2ind([nY, nX], yIdx(valid), xIdx(valid));
    countMap = reshape(accumarray(linIdx, 1, [nX*nY, 1]), [nY, nX]);
end
