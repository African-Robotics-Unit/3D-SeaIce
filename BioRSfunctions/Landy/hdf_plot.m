function hdf_plot(F,b)

% Plots the height distribution function of a 2-d surface profile F(x,y)
% using b bins.
%
% Input:
% F = surface heights
% b = number of bins         

format long

[hdf,bc] = hist(F,b); % histograms over columns
hdf = hdf/sum(hdf); % normalization to get height distribution function

% optional plotting
figure;
bar(bc,hdf);
xlabel('Surface Height')
ylabel('Probability')       
title('Plot of the Surface Height Distribution Function')

end
