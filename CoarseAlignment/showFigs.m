function [fig1,fig2] = showFigs(pc1,pc2)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
fig1=figure;
screenSize = get(0, 'ScreenSize'); % Get screen size
figWidth = 600; % Set figure width
figHeight = 400; % Set figure height
set(fig1, 'Position', [0, screenSize(4) - figHeight, figWidth, figHeight]);
% fig1 = uifigure;
% ax1 = uiaxes(fig1);
% ax1=pcshow(pc1, "AxesVisibility","on");
pcshow(pc1, "AxesVisibility","on")
xlabel('X');
ylabel('Y');
zlabel('Z');
title("pc1")
% fig2 = uifigure;
% ax2 = uiaxes(fig2);
% ax2=pcshow(pc2, "AxesVisibility","on");
fig2=figure;
set(fig2, 'Position', [screenSize(3) - figWidth, screenSize(4) - figHeight, figWidth, figHeight]);
pcshow(pc2, "AxesVisibility","on")
xlabel('X');
ylabel('Y');
zlabel('Z');
title("pc2")
end