function arrAfter = reorderPointCloud(arrBefore, fieldNotes, targetOrder)
%REORDERPOINTCLOUD Reorders pointcloud array based on alphabetical field
%notes
%
%   Inputs:
%       arrBefore: Unordered pointcloud array
%       fieldNotes: Alphabetical field notes order
%       targetOrder: Optional order for arrangement, default = 'ABCDEFGH'
%       representing 8 viewpoints
%
%   Outputs:
%       arrAfter: Ordered array of pointclouds
%   Example:
%   arrLiDAR_ordered=reorderPointCloud(arrLiDAR,'HABCDGFE');

    if nargin < 3
        targetOrder = 'ABCDEFGH';
    end

    arrAfter=arrBefore;
    % Reorder arrBefore according to targetOrder
    for i = 1:length(fieldNotes)
        index=strfind(fieldNotes,targetOrder(i));
        arrAfter(i) = arrBefore(index);
    end
end
