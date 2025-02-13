function arrAfter = reorderPointCloud(arrBefore, fieldNotes)
    % Ensure the fieldNotes has exactly 8 unique characters in A-H
    %if length(fieldNotes) ~= 8 || numel(unique(fieldNotes)) ~= 8 || ~all(ismember(fieldNotes, 'A':'H'))
      %  error('fieldNotes must be a unique combination of A-H characters with a length of 8');
   % end

    % Define the target order
    targetOrder = 'ABCDEFGH';

    % Create an empty cell array for the reordered point clouds
    %arrAfter = cell(1, 8);
    arrAfter=arrBefore;
    %pointCloudArray(numPointClouds) = 
    % Reorder arrBefore according to targetOrder
    for i = 1:8
        % Find the position in fieldNotes that corresponds to targetOrder(i)
        %index = find(fieldNotes == targetOrder(i));
        index=strfind(fieldNotes,targetOrder(i));
       % arrAfter{i} = arrBefore(index);
        arrAfter(i) = arrBefore(index);
    end
end
