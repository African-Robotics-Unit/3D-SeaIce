function stlCloud = samplePointsFromSTL(stlMesh, nPoints, unitScale)
% SAMPLEPOINTSFROMSTL Uniformly samples points from an STL triangle mesh
%
% Inputs:
%   stlMesh   - triangulation object from stlread()
%   nPoints   - number of points to sample (e.g. 50000)
%   unitScale - scale factor applied to points (default: 1/1000 for mm→m)
%
% Output:
%   stlCloud  - pointCloud object of sampled points

    if nargin < 3, unitScale = 1/1000; end

    verts  = stlMesh.Points * unitScale;
    faces  = stlMesh.ConnectivityList;

    % ── Compute area of each triangle ─────────────────────────────────────
    v0 = verts(faces(:,1), :);
    v1 = verts(faces(:,2), :);
    v2 = verts(faces(:,3), :);

    cross_vecs = cross(v1 - v0, v2 - v0, 2);
    areas      = 0.5 * vecnorm(cross_vecs, 2, 2);

    % ── Sample triangles weighted by area ─────────────────────────────────
    cumAreas = cumsum(areas) / sum(areas);
    r        = rand(nPoints, 1);
    triIdx   = arrayfun(@(x) find(cumAreas >= x, 1), r);

    % ── Random barycentric point within each triangle ─────────────────────
    r1 = rand(nPoints, 1);
    r2 = rand(nPoints, 1);
    outside     = r1 + r2 > 1;
    r1(outside) = 1 - r1(outside);
    r2(outside) = 1 - r2(outside);
    r3          = 1 - r1 - r2;

    pts = r1 .* verts(faces(triIdx,1), :) + ...
          r2 .* verts(faces(triIdx,2), :) + ...
          r3 .* verts(faces(triIdx,3), :);

    stlCloud = pointCloud(pts);

    fprintf('--- samplePointsFromSTL ---\n')
    fprintf('  Sampled : %d points\n',   nPoints)
    fprintf('  X range : [%.4f  %.4f] m\n', min(pts(:,1)), max(pts(:,1)))
    fprintf('  Y range : [%.4f  %.4f] m\n', min(pts(:,2)), max(pts(:,2)))
    fprintf('  Z range : [%.4f  %.4f] m\n', min(pts(:,3)), max(pts(:,3)))
end