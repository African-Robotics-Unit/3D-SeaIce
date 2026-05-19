function [icp] = runGliraICP(inputPath,exportPath)

%addpath(genpath('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\LiDAR Extraction'));
addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\GliraICP"));
if ~exist(exportPath, 'dir')
    % If the directory does not exist, create it
    mkdir(exportPath);
end

%Create ICP object
icp=globalICP('OutputFolder', exportPath);

%Extract files and add to ICP object
files = dir(strcat(inputPath,'*.ply'));
nviews = length(files);
fnames=cell(1,nviews);

for i=1:nviews
    filename = files(i).name;
    fnames{i}=filename;
    icp.addPC(strcat(inputPath,filename));
end

% ICPOptions.UniformSamplingDistance  = 0.2;
% ICPOptions.PlaneSearchRadius        = 0.1;
% ICPOptions.MaxRoughness             = 0.02;
%   ICPOptions.LogLevel                 = 'debug';
%   ICPOptions.Plot                     = true;
% % Run ICP!
% icp.runICP(ICPOptions);


end