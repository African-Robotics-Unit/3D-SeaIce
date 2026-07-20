function Zq = query_stl_z(gmTop, verts, topFaces, Xq, Yq)
% QUERY_STL_Z Finds exact Z on STL surface at each XY query point
% by computing barycentric coordinates within each triangle
%
% For each query point (x,y), finds which triangle contains it
% and interpolates Z exactly from the three vertices

    [nRows, nCols] = size(Xq);
    Zq = nan(nRows, nCols);

    % Precompute 2D triangle data (XY only)
    A = verts(topFaces(:,1), 1:2);   % Nx2
    B = verts(topFaces(:,2), 1:2);
    C = verts(topFaces(:,3), 1:2);

    zA = verts(topFaces(:,1), 3);
    zB = verts(topFaces(:,2), 3);
    zC = verts(topFaces(:,3), 3);

    for r = 1:nRows
        for c = 1:nCols
            P = [Xq(r,c), Yq(r,c)];

            % Barycentric coordinates for point P in each triangle
            v0 = C - A;   % Nx2
            v1 = B - A;   % Nx2
            v2 = repmat(P, size(A,1), 1) - A;

            dot00 = sum(v0.*v0, 2);
            dot01 = sum(v0.*v1, 2);
            dot02 = sum(v0.*v2, 2);
            dot11 = sum(v1.*v1, 2);
            dot12 = sum(v1.*v2, 2);

            inv  = 1 ./ (dot00.*dot11 - dot01.*dot01);
            u    = (dot11.*dot02 - dot01.*dot12) .* inv;
            v    = (dot00.*dot12 - dot01.*dot02) .* inv;

            % Point is inside triangle if u>=0, v>=0, u+v<=1
            inside = (u >= 0) & (v >= 0) & (u + v <= 1);

            if any(inside)
                idx = find(inside, 1);
                % Interpolate Z using barycentric coords
                Zq(r,c) = zA(idx)*(1-u(idx)-v(idx)) + zB(idx)*v(idx) + zC(idx)*u(idx);
            end
        end
    end
end