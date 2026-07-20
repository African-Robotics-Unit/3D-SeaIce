function [maxZCloud] = filterMaxZ_new(ptcloud, roundFactor)
    pts = ptcloud.Location; % Nx3 matrix [x, y, z]
    
    % Remove NaN points (if any)
    validIdx = all(~isnan(pts), 2);
    pts = pts(validIdx, :);
    
    % Preserve color if it exists
    hasColor = ~isempty(ptcloud.Color);
    if hasColor
        colors = ptcloud.Color;
        colors = colors(validIdx, :);
    end
    
    pts(:,1:2) = round(pts(:,1:2) / roundFactor) * roundFactor;
    
    % Convert to table
    if hasColor
        ptsTable = array2table([pts, double(colors)], ...
            'VariableNames', {'X', 'Y', 'Z', 'R', 'G', 'B'});
    else
        ptsTable = array2table(pts, 'VariableNames', {'X', 'Y', 'Z'});
    end
    
    % Compute max Z for each unique (X, Y)
    maxZTable = groupsummary(ptsTable, {'X', 'Y'}, "max", 'Z');
    
    % Extract new (X, Y, maxZ) points
    maxZPts = [maxZTable.X, maxZTable.Y, maxZTable.max_Z];
    
    if hasColor
        % Find the index of the original point with max Z for each group
        % by matching rounded X, Y, and max Z back to the pts array
        nGroups = height(maxZTable);
        colorOut = zeros(nGroups, 3, 'uint8');
        
        for i = 1:nGroups
            % Find rows matching this group's X, Y, and max Z
            match = pts(:,1) == maxZTable.X(i) & ...
                    pts(:,2) == maxZTable.Y(i) & ...
                    pts(:,3) == maxZTable.max_Z(i);
            idx = find(match, 1);
            colorOut(i, :) = colors(idx, :);
        end
        
        maxZCloud = pointCloud(maxZPts, 'Color', colorOut);
    else
        maxZCloud = pointCloud(maxZPts);
    end
end