function [resultsTable, arrDistances] = pc2pcGroupAnalysis(refCloud, arrAligned, outputFolder, outlierCutoff, cloudNames)
%PC2PCGROUPANALYSIS Compare a set of point clouds against one reference.
%
%   T = pc2pcGroupAnalysis(refCloud, arrAligned, outputFolder, outlierCutoff)
%   T = pc2pcGroupAnalysis(refCloud, arrAligned, outputFolder, outlierCutoff, cloudNames)
%
%   INPUTS
%     refCloud      pointCloud — shared reference surface
%     arrAligned    array of pointCloud objects to compare against refCloud
%                   (if you're storing them in a cell array instead, change
%                   arrAligned(i) to arrAligned{i} below)
%     outputFolder  folder to save one deviation figure per cloud + the
%                   summary CSV (created if it doesn't exist)
%     outlierCutoff distance threshold in MM — points with |distance|
%                   above this are flagged as outliers, excluded from
%                   stats, and shown as red X's in the figure
%     cloudNames    (optional) cell array of names, one per cloud — used
%                   for figure filenames and the CloudName column.
%                   Defaults to Cloud_01, Cloud_02, ...
%
%   OUTPUT
%     resultsTable  one row per cloud, sorted best fit (lowest RMS) first.
%                   Also written to <outputFolder>/pc2pc_group_summary.csv
%
%   Requires localPlaneFit.m and computeRugosity.m on the path.
%
%   Example:
%     arrAligned = [pc1, pc2, pc3];
%     names = {'run1','run2','run3'};
%     T = pc2pcGroupAnalysis(refCloud, arrAligned, 'results/', 5, names);

if nargin < 5 || isempty(cloudNames)
    cloudNames = arrayfun(@(i) sprintf('Cloud_%02d', i), 1:numel(arrAligned), 'UniformOutput', false);
end

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% ── PRECOMPUTE REFERENCE CLOUD (once, reused for every comparison) ───────
refPts    = double(refCloud.Location);
k_normals = 15;                          % neighbourhood size — tune to point density
tree_ref  = KDTreeSearcher(refPts);
idx_nbrs  = knnsearch(tree_ref, refPts, 'K', k_normals + 1);
[refNormals, refCentroids] = localPlaneFit(refPts, idx_nbrs);

flip = refNormals(:,3) < 0;              % orient normals to +Z — adjust if needed
refNormals(flip,:) = -refNormals(flip,:);

rugosity_ref = computeRugosity(refPts);

%% ── LOOP OVER ALIGNED CLOUDS ──────────────────────────────────────────────
nClouds = numel(arrAligned);
results = table();
arrDistances = cell(nClouds, 1);

for i = 1:nClouds
    name    = cloudNames{i};
    scanPts = double(arrAligned(i).Location);

    idx             = knnsearch(tree_ref, scanPts);
    nearestCentroid = refCentroids(idx,:);
    nearestNormal   = refNormals(idx,:);

    diffVec   = scanPts - nearestCentroid;
    distances = dot(diffVec, nearestNormal, 2);      % signed, metres
    arrDistances{i} = distances;
    d_mm_all  = distances * 1000;

    % ── outlier cutoff — excluded from stats, kept in the figure as flags ──
    outlier_mask = abs(d_mm_all) > outlierCutoff;
    n_total      = numel(d_mm_all);
    n_outliers   = sum(outlier_mask);
    d_mm         = d_mm_all(~outlier_mask);
    n_valid      = numel(d_mm);

    % ── figure ──────────────────────────────────────────────────────────
    fig = figure('Visible', 'off');
    scatter3(refPts(:,1), refPts(:,2), refPts(:,3), 2, [0.7 0.7 0.7], 'filled'); hold on
    validPts   = scanPts(~outlier_mask,:);
    outlierPts = scanPts(outlier_mask,:);
    scatter3(validPts(:,1), validPts(:,2), validPts(:,3), 8, d_mm, 'filled');
    if any(outlier_mask)
        scatter3(outlierPts(:,1), outlierPts(:,2), outlierPts(:,3), 15, 'rx');
    end
    colorbar; colormap(parula); caxis([-outlierCutoff, outlierCutoff]);  %#ok<CAXIS> — same colour scale across all clouds for fair comparison
    title(sprintf('%s vs reference — deviation (mm)', name), 'Interpreter', 'none');
    legend({'Reference', 'Valid points', 'Outliers'}, 'Location', 'best');
    view([1,1,1]); axis equal;
    savefig(fig,fullfile(outputFolder, sprintf('%s_deviation.fig', name)));
    saveas(fig, fullfile(outputFolder, sprintf('%s_deviation.png', name)));
    close(fig);

    % ── stats (valid points only) ──────────────────────────────────────
    if n_valid == 0
        warning('%s: every point exceeded outlierCutoff — stats set to NaN.', name);
        statRow   = nan(1,11);
        proud_pct = NaN; below_pct = NaN;
    else
        statRow = [mean(d_mm), median(d_mm), std(d_mm), rms(d_mm), skewness(d_mm), kurtosis(d_mm), ...
                   max(d_mm), min(d_mm), iqr(d_mm), prctile(abs(d_mm),95), prctile(abs(d_mm),99)];
        proud_pct = 100 * sum(d_mm > 0) / n_valid;
        below_pct = 100 * sum(d_mm < 0) / n_valid;
    end

    % ── rugosity — computed on the full scan cloud, independent of the
    %    deviation-outlier mask. It characterises the cloud's own surface
    %    geometry, not its registration accuracy against the reference. ───
    rugosity_scan = computeRugosity(scanPts);

    row = table({name}, n_total, n_valid, n_outliers, 100*n_outliers/n_total, ...
        statRow(1), statRow(2), statRow(3), statRow(4), statRow(5), statRow(6), ...
        statRow(7), statRow(8), statRow(9), statRow(10), statRow(11), ...
        proud_pct, below_pct, rugosity_ref, rugosity_scan, rugosity_scan - rugosity_ref, ...
        'VariableNames', {'CloudName','N_points','N_valid','N_outliers','Pct_outliers', ...
        'Mean_mm','Median_mm','Std_mm','RMS_mm','Skewness','Kurtosis','Max_mm','Min_mm', ...
        'IQR_mm','P95_abs_mm','P99_abs_mm','Pct_proud','Pct_below', ...
        'Rugosity_ref','Rugosity_scan','Rugosity_diff'});

    results = [results; row]; %#ok<AGROW>

    fprintf('[%d/%d] %s: RMS = %.3f mm | %d/%d outliers removed | rugosity diff = %+.4f\n', ...
        i, nClouds, name, statRow(4), n_outliers, n_total, rugosity_scan - rugosity_ref);
end

%% ── SORT + EXPORT ─────────────────────────────────────────────────────────
%resultsTable = sortrows(results, 'RMS_mm');   % best fit (lowest RMS) first
resultsTable = results;
csvPath = fullfile(outputFolder, 'pc2pc_group_summary.csv');
writetable(resultsTable, csvPath);
fprintf('\nSummary table written to: %s\n', csvPath);

end
