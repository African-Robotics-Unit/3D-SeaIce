function shiftedCloud = shiftCloudToSTLMaxZX(cloud, stlPoints)
% SHIFTCLOUDTOSTLMAXZX Shifts a point cloud so that:
%   1. Its max Z matches the STL max Z
%   2. The X position of the scan's max Z point matches the STL's max Z point X
%
% Inputs:
%   cloud     - pointCloud object or Nx3 matrix of [X Y Z] coordinates
%   stlPoints - Nx3 matrix of STL points (e.g. from stlread().Points / 1000)
%
% Output:
%   shiftedCloud - same type as input, with Z and X shifted

    % ── Get STL reference points ──────────────────────────────────────────
    [stlMaxZ, stlMaxIdx] = max(stlPoints(:,3));
    stlMaxZatX           = stlPoints(stlMaxIdx, 1);   % X of STL's peak point

    if isa(cloud, 'pointCloud')
        pts = double(cloud.Location);

        % ── Z shift: align max Z ──────────────────────────────────────────
        [scanMaxZ, scanMaxIdx] = max(pts(:,3));
        pts(:,3) = pts(:,3) - scanMaxZ + stlMaxZ;

        % ── X shift: align X at the peak point ───────────────────────────
        scanMaxZatX = pts(scanMaxIdx, 1);
        pts(:,1)    = pts(:,1) - scanMaxZatX + stlMaxZatX;

        shiftedCloud = pointCloud(pts, ...
            'Color',     cloud.Color, ...
            'Intensity', cloud.Intensity, ...
            'Normal',    cloud.Normal);

    elseif isnumeric(cloud) && size(cloud, 2) == 3
        % ── Z shift ───────────────────────────────────────────────────────
        [scanMaxZ, scanMaxIdx] = max(cloud(:,3));
        shiftedCloud           = cloud;
        shiftedCloud(:,3)      = cloud(:,3) - scanMaxZ + stlMaxZ;

        % ── X shift ───────────────────────────────────────────────────────
        scanMaxZatX        = shiftedCloud(scanMaxIdx, 1);
        shiftedCloud(:,1)  = shiftedCloud(:,1) - scanMaxZatX + stlMaxZatX;

    else
        error('Input must be a pointCloud object or an Nx3 numeric matrix.');
    end

    % ── Diagnostic output ─────────────────────────────────────────────────
    if isa(shiftedCloud, 'pointCloud')
        finalPts = double(shiftedCloud.Location);
    else
        finalPts = shiftedCloud;
    end
    [finalMaxZ, finalMaxIdx] = max(finalPts(:,3));
    fprintf('--- shiftCloudToSTLMaxZX ---\n')
    fprintf('  STL  peak Z : %.4f m  at X = %.4f m\n', stlMaxZ,   stlMaxZatX)
    fprintf('  Scan peak Z : %.4f m  at X = %.4f m\n', finalMaxZ, finalPts(finalMaxIdx,1))
end