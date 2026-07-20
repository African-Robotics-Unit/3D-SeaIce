function [normals, centroids] = localPlaneFit(pts, idxAll)
%LOCALPLANEFIT PCA-based local plane fit at each point.
%   normal(i,:)   = eigenvector of the smallest eigenvalue of the
%                   covariance of point i's k-nearest neighbours.
%   centroid(i,:) = mean of that neighbourhood — used as the plane's
%                   anchor point for signed distance calculations.
%
%   idxAll = knnsearch(tree, pts, 'K', k+1) — neighbour indices per point,
%   including the point itself.
%
%   Extracted out of pc2pcAnalysis so pc2pcGroupAnalysis can reuse it
%   without a second copy drifting out of sync. If pc2pcAnalysis.m still
%   has this embedded as a local function, delete that copy and let it
%   call this file instead.
    n = size(pts,1);
    normals   = zeros(n,3);
    centroids = zeros(n,3);
    for i = 1:n
        nbrs = pts(idxAll(i,:), :);
        centroids(i,:) = mean(nbrs,1);
        C = cov(nbrs);
        [V,D] = eig(C);
        [~, minIdx] = min(diag(D));
        normals(i,:) = V(:,minIdx)';
    end
end
