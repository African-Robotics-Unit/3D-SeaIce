addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\GliraICP"));
load("C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\AfterOutlierRemoved\ICP.mat");

x = 1:10; % Example x-values
mean_values = rand(1,10) * 10; % Example mean data
std_dev = rand(1,10) * 2; % Example standard deviation

errorbar(x, mean_values, std_dev, 'o-');
xlabel('X-axis label');
ylabel('Mean Value');
title('Mean with Standard Deviation Error Bars');
