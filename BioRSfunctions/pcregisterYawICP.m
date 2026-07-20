function [tform, rmse] = pcregisterYawICP(moving, fixed, varargin)
% PCREGISTERYAWICP  ICP constrained to Z-axis rotation only.
%
%   Rotation is restricted to the Z axis (no tip/tilt); translation is
%   free in X, Y, Z. Requires Statistics and Machine Learning Toolbox
%   (knnsearch).
%
%   [tform, rmse] = pcregisterYawICP(moving, fixed, ...
%       'MaxIterations', 200, 'Tolerance', 1e-6, 'InlierRatio', 0.9, ...
%       'InitialTransform', rigidtform3d(eye(3), [0 0 0]));
%
%   moving, fixed : pointCloud objects
%   tform         : rigidtform3d, maps moving -> fixed
%   rmse          : final nearest-neighbor RMSE

p = inputParser;
addParameter(p, 'MaxIterations', 100);
addParameter(p, 'Tolerance', 1e-6);        % RMSE change tolerance
addParameter(p, 'InlierRatio', 1.0);       % <1 to trim worst matches
addParameter(p, 'InitialTransform', rigidtform3d(eye(3), [0 0 0]));
parse(p, varargin{:});
maxIter    = p.Results.MaxIterations;
tol        = p.Results.Tolerance;
inlierFrac = p.Results.InlierRatio;
initT      = p.Results.InitialTransform;

X = double(moving.Location);   % N x 3, original moving points (never overwritten)
F = double(fixed.Location);    % M x 3

R_total = initT.R;
t_total = initT.Translation;
prevRMSE = Inf;

for iter = 1:maxIter
    movedPts = (R_total * X')' + t_total;

    % --- correspondences ---
    [idx, dists] = knnsearch(F, movedPts);
    corrFixed = F(idx, :);

    % --- optional trimming for robustness (partial overlap / outliers) ---
    if inlierFrac < 1.0
        nKeep = round(inlierFrac * numel(dists));
        [~, order] = sort(dists);
        keep = order(1:nKeep);
    else
        keep = 1:numel(dists);
    end

    src = movedPts(keep, :);
    tgt = corrFixed(keep, :);

    % --- Z translation: unconstrained by rotation, just the mean offset ---
    dz = mean(tgt(:,3) - src(:,3));

    % --- XY: 2D rigid alignment (rotation + translation) via SVD ---
    P = src(:,1:2);
    Q = tgt(:,1:2);
    Pc = P - mean(P);
    Qc = Q - mean(Q);
    H = Pc' * Qc;
    [U, ~, V] = svd(H);
    d = sign(det(V*U'));
    R2 = V * diag([1 d]) * U';               % 2x2 rotation about Z
    t2 = mean(Q)' - R2 * mean(P)';

    % --- assemble incremental 3D rigid transform ---
    Rinc = eye(3);
    Rinc(1:2,1:2) = R2;
    tinc = [t2; dz];

    % --- accumulate total transform ---
    R_total = Rinc * R_total;
    t_total = (Rinc * t_total')' + tinc';

    % --- convergence check ---
    movedPts = (R_total * X')' + t_total;
    [~, distsAll] = knnsearch(F, movedPts);
    rmse = sqrt(mean(distsAll.^2));

    if abs(prevRMSE - rmse) < tol
        break
    end
    prevRMSE = rmse;
end

tform = rigidtform3d(R_total, t_total);
end