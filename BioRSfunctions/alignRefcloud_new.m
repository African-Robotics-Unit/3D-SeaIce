function [arrAligned] = alignRefcloud_new(stlShape, arrScans)
% ALIGNREFCLOUD  Align each scan in arrScans to stlShape.
%
%   Pipeline per scan:
%     1. Coarse centroid match
%     2. Corner-snap initial guess (assumes known ~10x10cm rectangular
%        footprint) via minimum bounding rectangle + Kabsch 2D fit
%     3. Refine with constrained ICP: rotation restricted to Z axis only
%        (no tip/tilt), translation free in X, Y, Z
%
%   Usage:
%       arrAligned = alignRefcloud(stlShape, arrScans);

arrAligned = arrScans;

% --- Reference corners/plane from STL (computed once) ---
stlXY      = double(stlShape.Location(:,1:2));
stlCorners = orderCCW(minBoundingRectCorners(stlXY));
stlZ       = double(mean(stlShape.Location(:,3)));

for k = 1:numel(arrScans)
    scanDS = arrScans(k);

    %% Coarse alignment: match centroids
    stlCentroid  = mean(stlShape.Location);
    scanCentroid = mean(scanDS.Location);
    scanDS = pointCloud(scanDS.Location - scanCentroid + stlCentroid);

    %% Corner-snap initial guess (XY rotation/translation + Z offset)
    scanXY = double(scanDS.Location(:,1:2));
    scanZ  = double(mean(scanDS.Location(:,3)));

    [R2, t2, dz] = cornerAlignXY(scanXY, scanZ, stlCorners, stlZ);

    Rinit = eye(3);
    Rinit(1:2,1:2) = R2;
    tinit = [t2; dz]';
    initT = rigidtform3d(Rinit, tinit);

    %% Constrained ICP refinement (Z-axis rotation only)
    [tform, rmse] = pcregisterYawICP(scanDS, stlShape, ...
        'MaxIterations',    200, ...
        'Tolerance',        1e-6, ...
        'InlierRatio',      0.9, ...
        'InitialTransform', initT);
    % fprintf('Scan %d ICP RMSE: %.4f m\n', k, rmse);

    %% Apply transform to full scan
    scanAligned = pctransform(scanDS, tform);
    arrAligned(k) = scanAligned;
    %     figure
    %     pcDisplayPair(scanAligned, stlShape);
    %     title(letters(k));
end
end


%% ===================== Local helper functions =====================

function corners = minBoundingRectCorners(xy)
% Rotating calipers on convex hull -> 4 corners of min-area bounding rect
xy = double(xy);
k = convhull(xy(:,1), xy(:,2));
hull = xy(k(1:end-1), :);
n = size(hull,1);
minArea = Inf;
corners = [];
for i = 1:n
    p1 = hull(i,:);
    p2 = hull(mod(i,n)+1,:);
    edge = p2 - p1;
    edge = edge / norm(edge);
    perp = [-edge(2), edge(1)];
    proj1 = hull * edge';
    proj2 = hull * perp';
    lo1 = min(proj1); hi1 = max(proj1);
    lo2 = min(proj2); hi2 = max(proj2);
    area = (hi1-lo1) * (hi2-lo2);
    if area < minArea
        minArea = area;
        corners = [lo1*edge + lo2*perp;
                   hi1*edge + lo2*perp;
                   hi1*edge + hi2*perp;
                   lo1*edge + hi2*perp];
    end
end
end

function corners = orderCCW(corners)
c = mean(corners);
ang = atan2(corners(:,2)-c(2), corners(:,1)-c(1));
[~, idx] = sort(ang);
corners = corners(idx,:);
end

function [R, t, err] = kabsch2D(P, Q)
% Best-fit rotation+translation mapping P -> Q (both Nx2)
Pc = P - mean(P);
Qc = Q - mean(Q);
H = Pc' * Qc;
[U,~,V] = svd(H);
d = sign(det(V*U'));
R = V * diag([1 d]) * U';
t = mean(Q)' - R * mean(P)';
err = sum(sum(((R*P' + t)' - Q).^2));
end

function [R2, t2, dz] = cornerAlignXY(scanXY, scanZ, stlCorners, stlZ)
% scanXY: Nx2 detected scan points (XY only)
% stlCorners: 4x2 known STL corners, CCW ordered
scanCorners = orderCCW(minBoundingRectCorners(scanXY));
stlCornersCCW = orderCCW(stlCorners);

bestErr = Inf;
R2 = eye(2);
t2 = [0;0];
for shift = 0:3
    Pshift = circshift(scanCorners, shift, 1);
    [Rc, tc, err] = kabsch2D(Pshift, stlCornersCCW);
    if err < bestErr
        bestErr = err;
        R2 = Rc;
        t2 = tc;
    end
end
dz = stlZ - scanZ;
end