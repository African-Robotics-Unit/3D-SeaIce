clc;
clear;
close all;

restoredefaultpath;
rehash toolboxcache;

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..');

functionsPath = fullfile(projectRoot, 'functions');
gliraPath = fullfile(projectRoot, 'external', 'GliraICP');

% Add your own functions only
addpath(genpath(functionsPath));

% Do NOT add Glira yet
configpath = fullfile(projectRoot, 'config', 'p3_config.json');

% This uses MATLAB's pointCloud, so Glira must not be shadowing it
which pointCloud -all
arrLiDAR_ordered = preprocessClouds(configpath);

% Now add Glira only for fine alignment
addpath(genpath(gliraPath));
%rehash toolboxcache;

fineAlignment(configpath);

% Optional: remove Glira again afterwards
rmpath(genpath(gliraPath));
rehash toolboxcache;