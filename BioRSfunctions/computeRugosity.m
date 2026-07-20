function rugosity = computeRugosity(pts)
%COMPUTERUGOSITY  Surface rugosity = true 3D surface area / flat projected area.
%
%   rugosity = 1   → perfectly flat
%   rugosity > 1   → rougher / more topographically complex surface
%
%   METHOD:
%   1. Remove duplicate (x,y) points (Delaunay requires unique locations).
%   2. Triangulate the point cloud using ONLY its (x,y) footprint (Delaunay).
%   3. For each triangle, compute its TRUE 3D area (using x,y,z).
%   4. Compute the same triangle's FLAT area if it were projected onto XY.
%   5. rugosity = sum(3D areas) / sum(2D projected areas)
%
%   ASSUMPTION: "2.5D" surface — exactly one z per (x,y) location, e.g. a
%   downward-looking scan of ice/ground. NOT valid for overhangs or
%   vertical walls, since the XY-only triangulation can't represent them.

    % Step 1: remove duplicate (x,y) locations before triangulating.
    % If duplicates have different z, this keeps the FIRST occurrence only
    % — check n_removed below if this matters for your dataset.
    [xy_unique, uniqueIdx] = unique(pts(:,1:2), 'rows', 'stable');
    n_removed = size(pts,1) - size(xy_unique,1);
    if n_removed > 0
        fprintf('  computeRugosity: removed %d duplicate (x,y) points before triangulating.\n', n_removed);
    end
    pts = pts(uniqueIdx, :);

    x = pts(:,1);
    y = pts(:,2);

    % Step 2: triangulate footprint only (z ignored here)
    tri = delaunay(x, y);

    % Step 3: true 3D triangle areas (z included via full 3D edge vectors)
    edge1_3D = pts(tri(:,2), :) - pts(tri(:,1), :);
    edge2_3D = pts(tri(:,3), :) - pts(tri(:,1), :);
    area3D = 0.5 * sqrt(sum(cross(edge1_3D, edge2_3D, 2).^2, 2));

    % Step 4: flat projected areas (z dropped, XY only)
    edge1_2D = [x(tri(:,2)) - x(tri(:,1)), y(tri(:,2)) - y(tri(:,1))];
    edge2_2D = [x(tri(:,3)) - x(tri(:,1)), y(tri(:,3)) - y(tri(:,1))];
    area2D = 0.5 * abs(edge1_2D(:,1).*edge2_2D(:,2) - edge1_2D(:,2).*edge2_2D(:,1));

    % Step 5: ratio
    rugosity = sum(area3D) / sum(area2D);
end