function [fig1] = showCloud(ptCloud)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
fig1=figure;
% fig1 = uifigure;
% ax1 = uiaxes(fig1);
% ax1=pcshow(pc1, "AxesVisibility","on");
pcshow(ptCloud, "AxesVisibility","on")
xlabel('X');
ylabel('Y');
zlabel('Z');
end