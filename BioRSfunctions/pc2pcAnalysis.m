function pc2pcAnalysis(refCloud, pcAligned)
    %% ── POINT-CLOUD-TO-POINT-CLOUD DEVIATION ANALYSIS ────────────────────────
    % Replaces point-to-mesh (fastPoint2TriMesh) with point-to-cloud comparison.
    % Use when the STL mesh under-represents surface detail and you have a
    % dense point cloud instead (sampled from the STL, or from a separate scan).
    %
    % Key design change from point-to-mesh:
    %   - A mesh gives you a surface (triangles) and normals for free.
    %   - A point cloud gives you neither — both must be estimated locally
    %     from the neighbourhood around each reference point.
    %   - Method used here: nearest-neighbour lookup + local plane fit at the
    %     matched reference point (PCA over its k-nearest neighbours). This is
    %     the closest analogue to point-to-triangle distance — it reconstructs
    %     a small local surface patch instead of relying on triangulation.
    %     Plain point-to-point distance (no plane fit) is faster but "steps"
    %     with reference point spacing — see note in Section 2b.
    
    %% ── 1. LOAD REFERENCE + SCAN CLOUDS ───────────────────────────────────────
    refPts  = double(refCloud.Location);    % Nx3 — dense reference (was STL verts/faces)
    scanPts = double(pcAligned.Location);   % Mx3 — scan being compared
    
    %% ── 2. PRECOMPUTE REFERENCE ACCELERATION STRUCTURES (do once, reuse) ─────
    k_normals = 15;                          % neighbourhood size — tune to point density
    tree_ref  = KDTreeSearcher(refPts);
    idx_nbrs  = knnsearch(tree_ref, refPts, 'K', k_normals + 1);  % includes self
    
    [refNormals, refCentroids] = localPlaneFit(refPts, idx_nbrs);
    
    % Orient normals consistently — assumes surface roughly faces +Z.
    % Change axis/sign here if your reference geometry isn't oriented this way.
    flip = refNormals(:,3) < 0;
    refNormals(flip,:) = -refNormals(flip,:);
    
    % ── 2b. Faster alternative if you have Computer Vision / Lidar Toolbox ────
    % refNormals = pcnormals(pointCloud(refPts), k_normals);   % vectorised, much
    %                                                            % faster than the
    %                                                            % loop in localPlaneFit
    %                                                            % for large N
    % refCentroids = refPts;  % pcnormals doesn't give centroids — use nearest
    %                          % point itself if you go this route (see note below)
    
    %% ── 3. NEAREST NEIGHBOUR SEARCH ────────────────────────────────────────────
    idx = knnsearch(tree_ref, scanPts);
    
    %% ── 4. SIGNED POINT-TO-PLANE DISTANCE ──────────────────────────────────────
    fprintf('Computing point-to-cloud distances...\n');
    tic
    nearestCentroid = refCentroids(idx,:);
    nearestNormal   = refNormals(idx,:);
    
    diffVec     = scanPts - nearestCentroid;
    distances   = dot(diffVec, nearestNormal, 2);        % signed, metres
    project_pts = scanPts - distances .* nearestNormal;  % projection onto local plane
    outside     = distances > 0;                          % same convention as before:
                                                            % true = proud, false = below
    fprintf('Done in %.2f s. %d points processed.\n', toc, length(distances));
    
    pts = scanPts;
    figure()
    scatter3(refPts(:,1), refPts(:,2), refPts(:,3), 2, [0.7 0.7 0.7], 'filled'); hold on
    nearest_direction = project_pts - pts;
    scatter3(pts(:,1), pts(:,2), pts(:,3), 'bx');
    quiver3(pts(:,1), pts(:,2), pts(:,3), nearest_direction(:,1), nearest_direction(:,2), nearest_direction(:,3), 'k', 'AutoScale', 'off');
    legend({'Reference cloud','Points','Direction to surface'})
    view([1,1,1]);
    
    %% ── 5. BASIC STATS ───────────────────────────────────────────────────────
    d    = distances;          % shorthand
    d_mm = d * 1000;           % metres → mm for display
    
    n         = length(d);
    d_mean    = mean(d_mm);
    d_median  = median(d_mm);
    d_std     = std(d_mm);
    d_rms     = rms(d_mm);
    d_skew    = skewness(d_mm);
    d_kurt    = kurtosis(d_mm);
    d_max     = max(d_mm);
    d_min     = min(d_mm);
    d_p95     = prctile(abs(d_mm), 95);
    d_p99     = prctile(abs(d_mm), 99);
    d_iqr     = iqr(d_mm);
    
    outlier_mask = abs(d_mm - d_mean) > 3 * d_std;
    n_outliers   = sum(outlier_mask);
    
    proud_mask = d_mm > 0;
    below_mask = d_mm < 0;
    proud_pct  = 100 * sum(proud_mask) / n;
    below_pct  = 100 * sum(below_mask) / n;
    
    fprintf('\n========================================\n')
    fprintf('   POINT-TO-CLOUD STATISTICAL SUMMARY\n')
    fprintf('========================================\n')
    fprintf('  N points          : %d\n',    n)
    fprintf('  Mean              : %+.3f mm\n', d_mean)
    fprintf('  Median            : %+.3f mm\n', d_median)
    fprintf('  Std               : %.3f mm\n',  d_std)
    fprintf('  RMS               : %.3f mm\n',  d_rms)
    fprintf('  Skewness          : %.3f\n',     d_skew)
    fprintf('  Kurtosis          : %.3f\n',     d_kurt)
    fprintf('  Max (proud)       : %+.3f mm\n', d_max)
    fprintf('  Min (below)       : %+.3f mm\n', d_min)
    fprintf('  IQR               : %.3f mm\n',  d_iqr)
    fprintf('  95th pct |d|      : %.3f mm\n',  d_p95)
    fprintf('  99th pct |d|      : %.3f mm\n',  d_p99)
    fprintf('  Outliers (>3std)  : %d (%.1f%%)\n', n_outliers, 100*n_outliers/n)
    fprintf('  Points proud      : %.1f%%\n',   proud_pct)
    fprintf('  Points below      : %.1f%%\n',   below_pct)
    
    %% ── LOCAL FUNCTIONS ───────────────────────────────────────────────────────
    function [normals, centroids] = localPlaneFit(pts, idxAll)
        % PCA-based local plane fit at each point: normal = eigenvector of the
        % smallest eigenvalue of the covariance of its k-nearest neighbours.
        % centroid = mean of the neighbourhood (used as the plane's anchor
        % point for the signed distance calculation).
        n = size(pts,1);
        normals   = zeros(n,3);
        centroids = zeros(n,3);
        for i = 1:n
            nbrs = pts(idxAll(i,:), :);   % includes self — fine for centroid/plane fit
            centroids(i,:) = mean(nbrs,1);
            C = cov(nbrs);
            [V,D] = eig(C);
            [~, minIdx] = min(diag(D));
            normals(i,:) = V(:,minIdx)';
        end
    end
end