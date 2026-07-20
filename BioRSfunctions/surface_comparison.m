%% ============================================================
%  SURFACE COMPARISON: Scan vs STL
%  Computes roughness, rugosity, deviation statistics and plots
%  Inputs:  Xq, Yq, Zq        — STL interpolated grid
%           X_landy, Y_landy, F_landy — scan interpolated grid
%  All units assumed metres
%% ============================================================

%clear; clc; close all;

%% ── 0. LOAD YOUR DATA ────────────────────────────────────────────────────
% Replace this block with your actual workspace variables if running
% interactively — comment out if Xq/Yq/Zq etc. are already in workspace
%
% load('your_data.mat');   % must contain Xq, Yq, Zq, X_landy, Y_landy, F_landy

%% ── 1. COMMON GRID ───────────────────────────────────────────────────────
% Interpolate scan onto the STL grid so residuals are point-to-point valid

fprintf('=== Interpolating scan onto STL grid ===\n');

% Flatten scan grid to scattered points for griddata
xScan = X_landy(:);
yScan = Y_landy(:);
zScan = F_landy(:);

% Remove NaNs from scan before interpolating
valid = ~isnan(zScan);
xScan = xScan(valid);
yScan = yScan(valid);
zScan = zScan(valid);

% Interpolate scan onto STL grid
Zscan_on_stl = griddata(xScan, yScan, zScan, Xq, Yq, 'linear');

% Mask: only keep cells where BOTH surfaces have data
mask = ~isnan(Zq) & ~isnan(Zscan_on_stl);

Zstl_v  = Zq(mask);
Zscan_v = Zscan_on_stl(mask);
Xcommon = Xq(mask);
Ycommon = Yq(mask);

fprintf('  Common grid points: %d\n', sum(mask(:)));

%% ── 2. DETREND BOTH SURFACES ─────────────────────────────────────────────
% Fit and remove a mean plane so roughness is not contaminated by tilt/slope

fprintf('\n=== Detrending surfaces ===\n');

Zstl_dt  = detrend_surface(Xcommon, Ycommon, Zstl_v);
Zscan_dt = detrend_surface(Xcommon, Ycommon, Zscan_v);

%% ── 3. ROUGHNESS STATISTICS ──────────────────────────────────────────────
fprintf('\n=== Computing roughness statistics ===\n');

statsSTL  = compute_roughness(Zstl_dt,  'STL');
statsScan = compute_roughness(Zscan_dt, 'Scan');

%% ── 4. RUGOSITY ──────────────────────────────────────────────────────────
fprintf('\n=== Computing rugosity ===\n');

% Grid resolution (assumes uniform spacing)
res = mean(diff(Xq(1,:)));

rugSTL  = compute_rugosity(Xq,      Yq,      Zq,             mask, res, 'STL');
rugScan = compute_rugosity(X_landy, Y_landy, Zscan_on_stl,   mask, res, 'Scan');

%% ── 5. DEVIATION STATISTICS ──────────────────────────────────────────────
fprintf('\n=== Computing deviation (Scan - STL) ===\n');

dZ = Zscan_v - Zstl_v;   % residuals on common grid

devStats.mean   = mean(dZ);
devStats.std    = std(dZ);
devStats.rms    = sqrt(mean(dZ.^2));
devStats.max    = max(dZ);
devStats.min    = min(dZ);
devStats.skew   = skewness(dZ);
devStats.kurt   = kurtosis(dZ);
devStats.p95    = prctile(abs(dZ), 95);   % 95th percentile absolute deviation

fprintf('  Mean deviation  : %+.4f m\n', devStats.mean);
fprintf('  Std deviation   : %.4f m\n',  devStats.std);
fprintf('  RMS deviation   : %.4f m\n',  devStats.rms);
fprintf('  Max deviation   : %+.4f m\n', devStats.max);
fprintf('  Min deviation   : %+.4f m\n', devStats.min);
fprintf('  95th pct |dev|  : %.4f m\n',  devStats.p95);

%% ── 6. SUMMARY TABLE ────────────────────────────────────────────────────
fprintf('\n=== Building summary table ===\n');

paramNames = {'Ra (m)'; 'Rq (m)'; 'Rz (m)'; 'Skewness'; 'Kurtosis'; 'Rugosity'};
stlVals    = [statsSTL.Ra;  statsSTL.Rq;  statsSTL.Rz;
              statsSTL.skew; statsSTL.kurt; rugSTL];
scanVals   = [statsScan.Ra; statsScan.Rq; statsScan.Rz;
              statsScan.skew; statsScan.kurt; rugScan];
diffVals   = scanVals - stlVals;
pctDiff    = 100 * diffVals ./ abs(stlVals);

T = table(paramNames, stlVals, scanVals, diffVals, pctDiff, ...
    'VariableNames', {'Parameter','STL','Scan','Difference','PctDiff'});

fprintf('\n--- Surface Parameter Comparison ---\n');
disp(T);

devParamNames = {'Mean (m)'; 'Std (m)'; 'RMS (m)'; ...
                 'Max (m)'; 'Min (m)'; 'Skewness'; 'Kurtosis'; '95th pct |dZ| (m)'};
devVals = [devStats.mean; devStats.std; devStats.rms; devStats.max; ...
           devStats.min;  devStats.skew; devStats.kurt; devStats.p95];

T_dev = table(devParamNames, devVals, ...
    'VariableNames', {'DeviationParameter', 'Value'});

fprintf('\n--- Deviation Statistics (Scan - STL) ---\n');
disp(T_dev);

%% ── 7. TRANSECTS ─────────────────────────────────────────────────────────
% Extract transects at the midpoint of each axis

xVec = Xq(1,:);
yVec = Yq(:,1);

% Mid-row transect (constant Y ≈ centre)
[~, midRowIdx] = min(abs(yVec - median(yVec)));
xTransect      = xVec;
zStlTrans      = Zq(midRowIdx, :);
zScanTrans     = Zscan_on_stl(midRowIdx, :);

% Mid-col transect (constant X ≈ centre)
[~, midColIdx] = min(abs(xVec - median(xVec)));
yTransect      = yVec;
zStlTransY     = Zq(:, midColIdx);
zScanTransY    = Zscan_on_stl(:, midColIdx);

%% ── 8. PLOTS ─────────────────────────────────────────────────────────────
fprintf('\n=== Generating plots ===\n');

%% Figure 1: Surface overview (3 panels)
figure('Name','Surface Overview','Position',[50 50 1400 420]);

subplot(1,3,1);
surf(Xq, Yq, Zq, 'EdgeColor','none');
colormap(gca, turbo); colorbar;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('STL Design Surface'); axis tight; view(3);

subplot(1,3,2);
surf(X_landy, Y_landy, F_landy, 'EdgeColor','none');
colormap(gca, turbo); colorbar;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Scan Surface'); axis tight; view(3);

subplot(1,3,3);
dZ_grid          = nan(size(Xq));
dZ_grid(mask)    = dZ;
surf(Xq, Yq, dZ_grid, 'EdgeColor','none');
colormap(gca, bluewhitered(256)); colorbar;
clim([-1 1] * max(abs(dZ)));
xlabel('x (m)'); ylabel('y (m)'); zlabel('dZ (m)');
title('Deviation Map (Scan - STL)'); axis tight; view(3);

sgtitle('Surface Comparison Overview','FontSize',14,'FontWeight','bold');

%% Figure 2: Deviation map (top-down, publication-ready)
figure('Name','Deviation Map','Position',[50 520 700 560]);

dZ_grid       = nan(size(Xq));
dZ_grid(mask) = dZ;
pcolor(Xq, Yq, dZ_grid); shading flat;
colormap(bluewhitered(256)); c = colorbar;
c.Label.String = 'Scan − STL (m)';
clim([-1 1] * prctile(abs(dZ), 99));
xlabel('x (m)'); ylabel('y (m)');
title('Deviation Map (top view)');
axis equal tight;
hold on;
contour(Xq, Yq, dZ_grid, [0 0], 'k-', 'LineWidth', 1.5);   % zero-crossing
hold off;

%% Figure 3: Height distribution comparison
figure('Name','Height Distributions','Position',[770 520 700 560]);

[nStl,  eStl]  = histcounts(Zstl_dt,  60, 'Normalization','pdf');
[nScan, eScan] = histcounts(Zscan_dt, 60, 'Normalization','pdf');
[nDev,  eDev]  = histcounts(dZ,       60, 'Normalization','pdf');

subplot(2,1,1);
hold on;
stairs(eStl(1:end-1)*1000,  nStl,  'b-',  'LineWidth', 1.5, 'DisplayName', 'STL');
stairs(eScan(1:end-1)*1000, nScan, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Scan');
hold off;
xlabel('Detrended Z (mm)'); ylabel('Probability density');
title('Height Distribution: STL vs Scan');
legend; grid on;

subplot(2,1,2);
stairs(eDev(1:end-1)*1000, nDev, 'k-', 'LineWidth', 1.5);
xline(0,  'r--', 'LineWidth', 1.5, 'Label', 'Zero');
xline(devStats.mean*1000, 'b--', 'LineWidth', 1.2, ...
    'Label', sprintf('Mean=%.3f mm', devStats.mean*1000));
xlabel('Deviation (mm)'); ylabel('Probability density');
title('Deviation Distribution (Scan - STL)');
grid on;

sgtitle('Height Distributions','FontSize',14,'FontWeight','bold');

%% Figure 4: Transects
figure('Name','Transects','Position',[50 50 1100 500]);

subplot(2,2,1);
hold on;
plot(xTransect*1000, zStlTrans*1000,  'b-',  'LineWidth',1.5, 'DisplayName','STL');
plot(xTransect*1000, zScanTrans*1000, 'r--', 'LineWidth',1.2, 'DisplayName','Scan');
hold off;
xlabel('x (mm)'); ylabel('z (mm)');
title(sprintf('X-Transect at Y = %.3f m', yVec(midRowIdx)));
legend; grid on;

subplot(2,2,2);
hold on;
plot(yTransect*1000, zStlTransY*1000,  'b-',  'LineWidth',1.5, 'DisplayName','STL');
plot(yTransect*1000, zScanTransY*1000, 'r--', 'LineWidth',1.2, 'DisplayName','Scan');
hold off;
xlabel('y (mm)'); ylabel('z (mm)');
title(sprintf('Y-Transect at X = %.3f m', xVec(midColIdx)));
legend; grid on;

subplot(2,2,3);
dZ_transX = zScanTrans - zStlTrans;
plot(xTransect*1000, dZ_transX*1000, 'k-', 'LineWidth',1.2);
yline(0, 'r--'); yline(devStats.rms*1000,  'b:', 'LineWidth',1.2);
yline(-devStats.rms*1000, 'b:', 'LineWidth',1.2);
xlabel('x (mm)'); ylabel('dZ (mm)');
title('X-Transect Deviation'); grid on;

subplot(2,2,4);
dZ_transY = zScanTransY - zStlTransY;
plot(yTransect*1000, dZ_transY*1000, 'k-', 'LineWidth',1.2);
yline(0, 'r--'); yline(devStats.rms*1000,  'b:', 'LineWidth',1.2, 'Label','±RMS');
yline(-devStats.rms*1000, 'b:', 'LineWidth',1.2);
xlabel('y (mm)'); ylabel('dZ (mm)');
title('Y-Transect Deviation'); grid on;

sgtitle('Surface Transects: STL vs Scan','FontSize',14,'FontWeight','bold');

%% Figure 5: Parameter summary bar chart
figure('Name','Parameter Comparison','Position',[50 50 900 500]);

roughParams  = {'Ra','Rq','Rz'};
stlRough     = [statsSTL.Ra,  statsSTL.Rq,  statsSTL.Rz]  * 1000;   % → mm
scanRough    = [statsScan.Ra, statsScan.Rq, statsScan.Rz] * 1000;

subplot(1,3,1);
b = bar([stlRough; scanRough]', 'grouped');
b(1).FaceColor = [0.2 0.4 0.8];
b(2).FaceColor = [0.9 0.3 0.2];
set(gca,'XTickLabel', roughParams);
ylabel('mm'); title('Roughness Parameters');
legend({'STL','Scan'},'Location','best'); grid on;

subplot(1,3,2);
shapeParams  = {'Skewness','Kurtosis'};
stlShape     = [statsSTL.skew,  statsSTL.kurt];
scanShape    = [statsScan.skew, statsScan.kurt];
b2 = bar([stlShape; scanShape]', 'grouped');
b2(1).FaceColor = [0.2 0.4 0.8];
b2(2).FaceColor = [0.9 0.3 0.2];
set(gca,'XTickLabel', shapeParams);
title('Height Distribution Shape');
legend({'STL','Scan'},'Location','best'); grid on;

subplot(1,3,3);
b3 = bar([rugSTL, rugScan]);
b3(1).FaceColor = [0.2 0.4 0.8];
b3(2).FaceColor = [0.9 0.3 0.2];
set(gca,'XTickLabel', {'STL','Scan'});
ylabel('Rugosity (-)');
title('Rugosity'); grid on;
yline(1, 'k--', 'Flat reference', 'LineWidth',1.2);

sgtitle('Surface Parameter Comparison','FontSize',14,'FontWeight','bold');

fprintf('\n=== Done. ===\n');

%% =========================================================================
%%  HELPER FUNCTIONS
%% =========================================================================

function Zdt = detrend_surface(X, Y, Z)
% DETREND_SURFACE Fits and removes a least-squares mean plane from Z
% Inputs can be vectors (scattered) or matrices (will be flattened)
    X = X(:); Y = Y(:); Z = Z(:);
    valid = ~isnan(Z);
    A     = [X(valid), Y(valid), ones(sum(valid),1)];
    coeffs = A \ Z(valid);
    Zplane = X*coeffs(1) + Y*coeffs(2) + coeffs(3);
    Zdt    = Z - Zplane;
end

% ─────────────────────────────────────────────────────────────────────────

function stats = compute_roughness(Zdt, label)
% COMPUTE_ROUGHNESS Computes ISO-style roughness metrics on detrended Z
%   Zdt   - vector of detrended Z values
%   label - string label for display

    Zdt = Zdt(:);
    Zdt = Zdt(~isnan(Zdt));

    stats.Ra   = mean(abs(Zdt));
    stats.Rq   = std(Zdt);                          % RMS roughness
    stats.Rz   = max(Zdt) - min(Zdt);               % peak-to-valley
    stats.skew = skewness(Zdt);
    stats.kurt = kurtosis(Zdt);
    stats.mean = mean(Zdt);

    fprintf('  [%s] Ra=%.4f m  Rq=%.4f m  Rz=%.4f m  Skew=%.3f  Kurt=%.3f\n', ...
        label, stats.Ra, stats.Rq, stats.Rz, stats.skew, stats.kurt);
end

% ─────────────────────────────────────────────────────────────────────────

function rug = compute_rugosity(X, Y, Z, mask, res, label)
% COMPUTE_RUGOSITY Ratio of true 3D surface area to projected planar area
%   Uses the triangle-pair method on each 2x2 cell of the grid
%   X, Y, Z  - full grid matrices
%   mask     - logical mask of valid cells
%   res      - grid resolution in metres
%   label    - string label for display

    [nRows, nCols] = size(Z);
    trueArea  = 0;
    planArea  = 0;

    for r = 1:nRows-1
        for c = 1:nCols-1
            % Corners of this cell
            p00 = [X(r,c),   Y(r,c),   Z(r,c)];
            p10 = [X(r+1,c), Y(r+1,c), Z(r+1,c)];
            p01 = [X(r,c+1), Y(r,c+1), Z(r,c+1)];
            p11 = [X(r+1,c+1), Y(r+1,c+1), Z(r+1,c+1)];

            % Skip if any corner is NaN or outside mask
            if any(isnan([p00(3), p10(3), p01(3), p11(3)]))
                continue;
            end

            % Split cell into 2 triangles and sum areas
            a1 = tri_area(p00, p10, p01);
            a2 = tri_area(p10, p11, p01);
            trueArea = trueArea + a1 + a2;
            planArea = planArea + res^2;        % flat projected area of cell
        end
    end

    if planArea == 0
        rug = NaN;
        warning('compute_rugosity: no valid cells found for %s', label);
    else
        rug = trueArea / planArea;
    end

    fprintf('  [%s] True area=%.4f m²  Plan area=%.4f m²  Rugosity=%.4f\n', ...
        label, trueArea, planArea, rug);
end

% ─────────────────────────────────────────────────────────────────────────

function a = tri_area(p1, p2, p3)
% TRI_AREA Area of a triangle defined by three 3D points
    v1 = p2 - p1;
    v2 = p3 - p1;
    a  = 0.5 * norm(cross(v1, v2));
end

% ─────────────────────────────────────────────────────────────────────────

function cmap = bluewhitered(n)
% BLUEWHITERED Blue-white-red diverging colormap symmetric around 0
    if nargin < 1, n = 256; end
    half = floor(n/2);
    b2w  = [linspace(0,1,half)', linspace(0,1,half)', ones(half,1)];
    w2r  = [ones(half,1), linspace(1,0,half)', linspace(1,0,half)'];
    cmap = [b2w; w2r];
end
