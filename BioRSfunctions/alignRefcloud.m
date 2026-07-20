function [arrAligned] = alignRefcloud(stlShape,arrScans)
%stlShape = arrShapesScaled(2);
arrAligned = arrScans;
for k=1:5
    scanDS = arrScans(k);
    
    %% Coarse alignment first: match centroids + scale
    % (only needed if units differ or clouds are far apart)
    stlCentroid  = mean(stlShape.Location);
    scanCentroid = mean(scanDS.Location);
    scanDS = pointCloud(scanDS.Location - scanCentroid + stlCentroid);

%     %% ICP: align scan to STL
%     [tform, ~, rmse] = pcregistericp(scanDS, stlShape, ...
%         'MaxIterations', 200, ...
%         'Tolerance',     [1e-6 1e-6]);
   % fprintf('ICP RMSE: %.4f m\n', rmse);

    [tform, rmse] = pcregisterYawICP(scanDS, stlShape, ...
        'MaxIterations', 200, ...
        'Tolerance', 1e-6, ...
        'InlierRatio', 1.0);
    
    %% Apply transform to FULLcan
    scanAligned = pctransform(scanDS, tform);
    arrAligned(k) = scanAligned;
%     figure
%     %pcDisplay(scanDS)
%     pcDisplayPair(scanAligned, stlShape);
%     title(letters(k));
end
end