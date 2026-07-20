function statsTable = pc2meshBatchAnalysis(arrAlignedPyramid, stlPyramid, pcRef, outputFolder, seriesName, names, use_parallel)
% PC2MESHBATCHANALYSIS Compare each scan in arrAlignedPyramid to stlPyramid
%
% Inputs:
%   arrAlignedPyramid - array/cell of aligned pointCloud objects
%   stlPyramid         - triangulation object (STL reference mesh)
%   pcRef               - pointCloud object of the reference STL, used to
%                          generate its own XY/XZ scatter plots for comparison
%   outputFolder        - folder to save figures (created if missing)
%   seriesName           - string, e.g. "pyramid" — used as CSV/MAT filename base
%   names                - array of labels per scan, e.g. ["A","B","C","D","E"]
%   use_parallel         - 1 to use Parallel Computing Toolbox, 0 otherwise
%
% Output:
%   statsTable - table of per-scan stats, in original scan order (unsorted)

if nargin < 7
    use_parallel = 0;
end

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

nScans = numel(arrAlignedPyramid);

if numel(names) ~= nScans
    error('names must have the same length as arrAlignedPyramid (%d).', nScans);
end

% Build mesh inputs once, reused for every scan
gm = stlPyramid;
verts = gm.Points / 1000;
faces = gm.ConnectivityList;

inputs.faces = faces;
inputs.nodes = verts;
% face_normals, face_mean_nodes, tree_model left out
% — computed internally by fastPoint2TriMesh

% ── STL REFERENCE RUGOSITY (computed once, same for every comparison) ───
rugosity_stl = computeRugosity(verts);
fprintf('STL reference rugosity: %.4f\n', rugosity_stl);

% ── STL REFERENCE XY / XZ SCATTER PLOTS ──────────────────────────────────
refPts = double(pcRef.Location);
plotScatterViews(refPts, "STL_" + seriesName, outputFolder);

results = cell(nScans, 1);

for i = 1:nScans

    label = string(names(i));
    baseName = label + "_" + seriesName;

    scanAligned = arrAlignedPyramid(i);
    scanPts = double(scanAligned.Location);

    fprintf('\n[%d/%d] (%s) Computing point-to-mesh distances...\n', i, nScans, label);
    tic
    [distances, project_pts, ~] = fastPoint2TriMesh(inputs, scanPts, use_parallel);
    elapsed = toc;
    fprintf('  Done in %.2f s. %d points processed.\n', elapsed, length(distances));

    % ── DEVIATION STATS ──────────────────────────────────────────────────
    d = distances;
    d_mm = d * 1000;
    n = length(d_mm);

    d_mean   = mean(d_mm);
    d_median = median(d_mm);
    d_std    = std(d_mm);
    d_rms    = rms(d_mm);
    d_skew   = skewness(d_mm);
    d_kurt   = kurtosis(d_mm);
    d_max    = max(d_mm);
    d_min    = min(d_mm);
    d_p95    = prctile(abs(d_mm), 95);
    d_p99    = prctile(abs(d_mm), 99);
    d_iqr    = iqr(d_mm);

    outlier_mask = abs(d_mm - d_mean) > 3 * d_std;
    n_outliers   = sum(outlier_mask);

    proud_mask = d_mm > 0;
    below_mask = d_mm < 0;
    proud_pct  = 100 * sum(proud_mask) / n;
    below_pct  = 100 * sum(below_mask) / n;

    fprintf('  Mean: %+.3f mm | RMS: %.3f mm | Std: %.3f mm\n', d_mean, d_rms, d_std);

    % ── RUGOSITY COMPARISON ──────────────────────────────────────────────
    rugosity_scan = computeRugosity(scanPts);
    rugosity_diff = rugosity_scan - rugosity_stl;
    rugosity_pctdiff = 100 * rugosity_diff / rugosity_stl;

    fprintf('  Rugosity — STL: %.4f | Scan: %.4f | Diff: %+.4f (%+.2f%%)\n', ...
        rugosity_stl, rugosity_scan, rugosity_diff, rugosity_pctdiff);

    % ── FIGURE 1: DEVIATION QUIVER PLOT ──────────────────────────────────
    fig1 = figure('Visible', 'off');
    patch('Faces', faces, 'Vertices', verts, 'FaceColor', 'r', 'FaceAlpha', .25, 'EdgeAlpha', .2);
    hold on
    nearest_direction = project_pts - scanPts;
    scatter3(scanPts(:,1), scanPts(:,2), scanPts(:,3), 'bx');
    quiver3(scanPts(:,1), scanPts(:,2), scanPts(:,3), ...
        nearest_direction(:,1), nearest_direction(:,2), nearest_direction(:,3), 'k', 'AutoScale', 'off');
    legend({'Object', 'Points', 'Direction to Surface'});
    view([1,1,1]);
    title(sprintf('%s — Point-to-Mesh Deviation', label));

    pngPath = fullfile(outputFolder, baseName + ".png");
    figPath = fullfile(outputFolder, baseName + ".fig");
    exportgraphics(fig1, pngPath, 'Resolution', 300);
    savefig(fig1, figPath);
    close(fig1);

    % ── FIGURES 2 & 3: XY (colored by Z) and XZ SCATTER ───────────────────
    plotScatterViews(scanPts, baseName, outputFolder);

    % ── STORE ROW ──────────────────────────────────────────────────────
    results{i} = struct( ...
        'Label',            label, ...
        'ScanIndex',        i, ...
        'N',                n, ...
        'Mean_mm',          d_mean, ...
        'Median_mm',        d_median, ...
        'Std_mm',           d_std, ...
        'RMS_mm',           d_rms, ...
        'Skewness',         d_skew, ...
        'Kurtosis',         d_kurt, ...
        'Max_mm',           d_max, ...
        'Min_mm',           d_min, ...
        'IQR_mm',           d_iqr, ...
        'P95_abs_mm',       d_p95, ...
        'P99_abs_mm',       d_p99, ...
        'NOutliers',        n_outliers, ...
        'OutlierPct',       100*n_outliers/n, ...
        'ProudPct',         proud_pct, ...
        'BelowPct',         below_pct, ...
        'Rugosity_STL',     rugosity_stl, ...
        'Rugosity_Scan',    rugosity_scan, ...
        'Rugosity_Diff',    rugosity_diff, ...
        'Rugosity_PctDiff', rugosity_pctdiff, ...
        'ComputeTime_s',    elapsed);
end

statsTable = struct2table([results{:}]);
% Row order preserved as original scan order — no sorting applied.

% Save table alongside figures, named using seriesName
csvPath = fullfile(outputFolder, seriesName + "_summary.csv");
matPath = fullfile(outputFolder, seriesName + "_summary.mat");
writetable(statsTable, csvPath);
save(matPath, 'statsTable');

fprintf('\n========================================\n');
fprintf('  Batch complete: %d scans processed\n', nScans);
fprintf('  Summary saved to: %s\n', outputFolder);
fprintf('========================================\n');

end


function plotScatterViews(pts, baseName, outputFolder)
%PLOTSCATTERVIEWS Save XY (colored by Z) and XZ scatter plots as PNG.
    xvals = pts(:,1);
    yvals = pts(:,2);
    zvals = pts(:,3);

    % XY view, colored by Z
    fig = figure('Visible', 'off');
    scatter(xvals, yvals, 5, zvals, 'filled');
    colormap(fig, parula);
    cb = colorbar;
    cb.Label.String = 'Z (m)';
    title("XY view: " + baseName);
    xlim([0, 0.1]);
    ylim([0, 0.1]);

    xyPngPath = fullfile(outputFolder, baseName + "_xy.png");
    exportgraphics(fig, xyPngPath, 'Resolution', 300);
    close(fig);

    % XZ view
    fig = figure('Visible', 'off');
    scatter(xvals, zvals, 5, 'filled');
    title("XZ view: " + baseName);
    xlim([0, 0.1]);
    ylim([0, 0.1]);

    xzPngPath = fullfile(outputFolder, baseName + "_xz.png");
    exportgraphics(fig, xzPngPath, 'Resolution', 300);
    close(fig);
end