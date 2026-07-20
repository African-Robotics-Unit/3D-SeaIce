function stlCloud = sampleUniformSTL(stlMesh, nPoints, unitScale)
% SAMPLEUNIFORMSTL Uniformly samples points from an STL triangle mesh
% using deterministic low-discrepancy sampling instead of random sampling.
%
% Inputs:
%   stlMesh   - triangulation object from stlread()
%   nPoints   - number of points to sample (e.g. 50000)
%   unitScale - scale factor applied to points (default: 1/1000 for mm→m)
%
% Output:
%   stlCloud  - pointCloud object of uniformly sampled points

if nargin < 3, unitScale = 1/1000; end
    verts  = stlMesh.Points * unitScale;
    faces  = stlMesh.ConnectivityList;

% ── Compute area of each triangle ─────────────────────────────────────
    v0 = verts(faces(:,1), :);
    v1 = verts(faces(:,2), :);
    v2 = verts(faces(:,3), :);
    cross_vecs = cross(v1 - v0, v2 - v0, 2);
    areas      = 0.5 * vecnorm(cross_vecs, 2, 2);

% ── Deterministic (systematic) triangle selection weighted by area ────
% Evenly spaced quantiles walk the area-weighted CDF, replacing rand()
% so triangle counts track area proportion without random clustering.
    cumAreas = cumsum(areas) / sum(areas);
    q        = ((0:nPoints-1)' + 0.5) / nPoints;
    triIdx   = arrayfun(@(x) find(cumAreas >= x, 1), q);

% ── Low-discrepancy (Halton) barycentric coordinates ───────────────────
% Halton sequence (bases 2 and 3) replaces rand() for in-triangle
% placement, giving uniform space-filling coverage instead of random
% scatter/clumping.
    h1 = haltonSeq(nPoints, 2);
    h2 = haltonSeq(nPoints, 3);
    outside      = h1 + h2 > 1;
    h1(outside)  = 1 - h1(outside);
    h2(outside)  = 1 - h2(outside);
    r1 = h1;
    r2 = h2;
    r3 = 1 - r1 - r2;

    pts = r1 .* verts(faces(triIdx,1), :) + ...
          r2 .* verts(faces(triIdx,2), :) + ...
          r3 .* verts(faces(triIdx,3), :);

    stlCloud = pointCloud(pts);

    fprintf('--- sampleUniformSTL ---\n')
    fprintf('  Sampled : %d points (uniform/deterministic)\n', nPoints)
    fprintf('  X range : [%.4f  %.4f] m\n', min(pts(:,1)), max(pts(:,1)))
    fprintf('  Y range : [%.4f  %.4f] m\n', min(pts(:,2)), max(pts(:,2)))
    fprintf('  Z range : [%.4f  %.4f] m\n', min(pts(:,3)), max(pts(:,3)))
end

function h = haltonSeq(n, base)
% HALTONSEQ First n terms of the Halton low-discrepancy sequence.
    h = zeros(n, 1);
    for i = 1:n
        f   = 1;
        r   = 0;
        idx = i;
        while idx > 0
            f   = f / base;
            r   = r + f * mod(idx, base);
            idx = floor(idx / base);
        end
        h(i) = r;
    end
end