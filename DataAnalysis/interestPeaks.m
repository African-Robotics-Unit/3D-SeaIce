function [output] = interestPeaks(F,X,Y,n,cutoff)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
X_interest_all = [];
Y_interest_all = [];
Z_interest_all = [];

trough_X_interest_all = [];
trough_Y_interest_all = [];
trough_Z_interest_all = [];


peak2_X_interest_all = [];
peak2_Y_interest_all = [];
peak2_Z_interest_all = [];
arrPeaks=[];
arrPeaksAll=zeros(1,n);
arrTroughs=[];
for i = 1:n
    subsample = F(i,:); % Extract current row of F
    validIdx = ~isnan(subsample); % Get logical index of non-NaN values

    % Store original indices before removing NaNs
    originalIndices = find(validIdx); 
    zsample = subsample(validIdx); % Remove NaNs but keep track of original positions

    % Local extrema detection
    localMaxima = islocalmax(zsample);
    localMaximaIndices = find(localMaxima);
    localMini = islocalmin(zsample);
    localMiniIndices = find(localMini);
    
    [zMaxSorted, sortIdx] = sort(zsample(localMaximaIndices), 'descend'); 
    %[zMaxSorted, sortIdx] = sort(zsample(localMaximaIndices)); 
    idxMaxSorted = localMaximaIndices(sortIdx);
    %arrDiff = zeros(1, length(localMiniIndices));

    topValidx=idxMaxSorted(1);

    found=false;
    arrDiff=[];
    for k = idxMaxSorted(1):-1:1
        % Check if k is in localMiniIndices
        if ismember(k, localMiniIndices)
            diffval = zMaxSorted(1) - zsample(k);
            diffpercent=diffval/zMaxSorted(1)*100;
            arrDiff=[arrDiff,diffpercent];
            %if diffval > zMaxSorted(1) / 4
            if diffpercent >=cutoff
                found = true;
                idxFound = k;
                break; % Exit loop once found
            end
        end
    end
    %arrPeaksAll(i)=sortedPOI(floorIndex);
    arrPeaksAll(i)=0;
    if found
        trough_X_interest_all = [trough_X_interest_all, X(i, originalIndices(idxFound))];
        trough_Y_interest_all = [trough_Y_interest_all, Y(i, originalIndices(idxFound))];
        trough_Z_interest_all = [trough_Z_interest_all, zsample(idxFound)];

        validMaxIdx = localMaximaIndices(localMaximaIndices <= idxFound);
    
        if ~isempty(validMaxIdx) % Ensure there are valid peaks
            [maxPeakValue, maxPeakIdx] = max(zsample(validMaxIdx)); % Find max peak
            idxMaxPeak = validMaxIdx(maxPeakIdx); % Get corresponding index
        end

            peakX = X(i, originalIndices(topValidx));
            peakY = Y(i, originalIndices(topValidx));
            peakZ = zMaxSorted(1);

            troughX=X(i, originalIndices(idxFound));
            troughY=Y(i, originalIndices(idxFound));
            troughZ=zsample(idxFound);

            arrPeaks = [arrPeaks; peakX, peakY, peakZ];
            arrPeaksAll(i)=peakZ;
            arrTroughs = [arrTroughs; troughX, troughY, troughZ];

        peak2_X_interest_all = [peak2_X_interest_all, X(i, originalIndices(idxMaxPeak))];
        peak2_Y_interest_all = [peak2_Y_interest_all, Y(i, originalIndices(idxMaxPeak))];
        peak2_Z_interest_all = [peak2_Z_interest_all, maxPeakValue];
    end

    X_interest_all = [X_interest_all, X(i, originalIndices(topValidx))];
    Y_interest_all = [Y_interest_all, Y(i, originalIndices(topValidx))];
    Z_interest_all = [Z_interest_all, zMaxSorted(1)];
end

arrGrad = []; % Initialize arrGrad to store gradients
arrAngles = [];
for j = 1:size(arrPeaks, 1)
    peak = arrPeaks(j, :);
    trough = arrTroughs(j, :);
    
    % Calculate the X-Z gradient for the current peak and trough
    % Gradient formula: (Z_peak - Z_trough) / (X_peak - X_trough)
    deltaZ = peak(3) - trough(3);
    deltaX = peak(1) - trough(1);
    
    % Avoid division by zero
    if deltaX ~= 0
        grad = deltaZ / deltaX; % X-Z gradient
        angle_rad = atan(grad);
        % Convert angle from radians to degrees
        angle_deg = rad2deg(angle_rad);
    else
        grad = NaN; % Assign NaN if division by zero
        angle_deg = NaN;
    end
    
    % Store the calculated gradient
    arrGrad = [arrGrad; grad];
    arrAngles = [arrAngles; angle_deg];
end

avgAngle = mean(arrAngles, 'omitnan');

% surf(Y, X, F,'DisplayName', 'Surface');
% %shading interp; 
% hold on;

% scatter3(Y_interest_all, X_interest_all, Z_interest_all, 15, "red", "filled",'DisplayName', 'Max points');
%scatter3(trough_X_interest_all, trough_Y_interest_all, trough_Z_interest_all, 10, "blue", "filled");
%scatter3(peak2_X_interest_all,peak2_Y_interest_all, peak2_Z_interest_all, 10, "black", "filled");

for j = 1:size(arrPeaks, 1)
    peak = arrPeaks(j, :);
    trough = arrTroughs(j, :);
    %plot3([peak(1), trough(1)], [peak(2), trough(2)], [peak(3), trough(3)], 'k-', 'LineWidth', 1.5);
end

vars = whos;
% Create a struct and add each variable to it
output = struct();
for k = 1:length(vars)
    name = vars(k).name;
    output.(name) = eval(name);
end
% view(2)
% hold off;
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z');
% axis equal
% %title('Interest Points Overlaid on Surf Plot');
% c = colorbar('southoutside'); % Position colorbar below the plot
% c.Label.String = 'Z (m)';   % Set label text
% legend('Location', 'eastoutside'); % Moves it outside on the right
% grid on;
end