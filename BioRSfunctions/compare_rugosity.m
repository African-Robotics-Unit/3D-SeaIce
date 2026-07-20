function compare_rugosity(pts1, pts2, label1, label2)
% pts1, pts2: Nx3 point clouds [x y z]
% label1, label2: strings for titles/legend

if nargin < 3, label1 = 'Cloud 1'; end
if nargin < 4, label2 = 'Cloud 2'; end

[tri1, rug1, localRug1] = compute_rugosity(pts1);
[tri2, rug2, localRug2] = compute_rugosity(pts2);

% shared color scale for fair visual comparison
climVals = [min([localRug1; localRug2]), max([localRug1; localRug2])];

figure('Position', [100 100 1200 500]);

subplot(1,2,1);
plot_triangulation(pts1, tri1, localRug1, label1, climVals);

subplot(1,2,2);
plot_triangulation(pts2, tri2, localRug2, label2, climVals);

sgtitle(sprintf('Rugosity: %s = %.4f | %s = %.4f', label1, rug1, label2, rug2));

% bar comparison
figure;
bar([rug1, rug2]);
set(gca, 'XTickLabel', {label1, label2});
ylabel('Rugosity (3D area / 2D projected area)');
title('Rugosity Comparison');
grid on;

fprintf('%s rugosity: %.6f\n', label1, rug1);
fprintf('%s rugosity: %.6f\n', label2, rug2);
fprintf('Difference: %.6f (%.2f%%)\n', rug2-rug1, 100*(rug2-rug1)/rug1);

end

function [tri, rugosity, localRug] = compute_rugosity(pts)
x = pts(:,1); y = pts(:,2);
tri = delaunay(x, y);

v1 = pts(tri(:,2),:) - pts(tri(:,1),:);
v2 = pts(tri(:,3),:) - pts(tri(:,1),:);
area3D = 0.5 * sqrt(sum(cross(v1, v2, 2).^2, 2));

v1_2d = [x(tri(:,2))-x(tri(:,1)), y(tri(:,2))-y(tri(:,1))];
v2_2d = [x(tri(:,3))-x(tri(:,1)), y(tri(:,3))-y(tri(:,1))];
area2D = 0.5 * abs(v1_2d(:,1).*v2_2d(:,2) - v1_2d(:,2).*v2_2d(:,1));

localRug = area3D ./ area2D;   % per-triangle rugosity
rugosity = sum(area3D) / sum(area2D);
end

function plot_triangulation(pts, tri, localRug, titleStr, climVals)
trisurf(tri, pts(:,1), pts(:,2), pts(:,3), localRug, ...
    'EdgeColor', 'k', 'EdgeAlpha', 0.15);
shading interp;
colormap(gca, turbo);
colorbar;
axis equal;
view(2);   % swap to view(3) for a tilted 3D view instead of top-down
xlabel('X'); ylabel('Y'); zlabel('Z');
title(titleStr);
end