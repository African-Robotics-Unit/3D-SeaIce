function [points] = getPoints(ptCloud)
%GETPOINTS Summary of this function goes here
%   Detailed explanation goes here
 figure;
 pcshow(ptCloud, "AxesVisibility","on")
xlabel('X');
ylabel('Y');
zlabel('Z');
title("pcTest")
dcm = datacursormode;
dcm.Enable = 'on';
dcm.DisplayStyle = 'window';

end

