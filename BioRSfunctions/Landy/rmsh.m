function [h] = rmsh(F)
%
% calculates the RMS height of a 2-d surface of the form F(x,y) in x and
% y directions
%
% INPUT:
% F = regularly gridded surface model (DEM)
%
% Output:
% h = RMS height in meters

format long

h=mean(std(F));

end