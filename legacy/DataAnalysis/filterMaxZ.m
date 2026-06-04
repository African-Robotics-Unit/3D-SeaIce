function [maxZCloud] = filterMaxZ(ptcloud, roundFactor)
pts = ptcloud.Location; % Nx3 matrix [x, y, z]

% Remove NaN points (if any)
validIdx = all(~isnan(pts), 2);
pts = pts(validIdx, :);

%roundFactor = 0.02;
pts(:,1:2) = round(pts(:,1:2) / roundFactor) * roundFactor;

% Convert to table
ptsTable = array2table(pts, 'VariableNames', {'X', 'Y', 'Z'});

% Compute max Z for each unique (X, Y)
maxZTable = groupsummary(ptsTable, {'X', 'Y'}, "max", 'Z');

% Extract new (X, Y, maxZ) points
maxZPts = [maxZTable.X, maxZTable.Y, maxZTable.max_Z];

% Create new pointCloud object
maxZCloud = pointCloud(maxZPts);

end