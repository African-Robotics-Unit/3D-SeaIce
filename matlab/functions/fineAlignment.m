function fineAlignment(configpath)
% Get current script folder
    scriptDir = fileparts(mfilename('fullpath'));
    
    % Move up to project root
    projectRoot = fullfile(scriptDir, '..');
    
    % External library path
    gliraPath = fullfile(projectRoot, 'external', 'GliraICP');
    
    addpath(genpath(gliraPath));
    
    txt = fileread(configpath);
    params = jsondecode(txt);
    outputFolder = params.outputFolder;
    beforeICPpath=strcat(outputFolder,params.preprocessing.beforeICPpath);
    afterICPpath=strcat(outputFolder,params.preprocessing.afterICPpath);
    
    icptest = runGliraICP(beforeICPpath, afterICPpath);

    % ICP options
    ICPOptions.NoOfTransfParam         = params.ICPOptions.NoOfTransfParam;
    ICPOptions.UniformSamplingDistance = params.ICPOptions.UniformSamplingDistance;
    ICPOptions.PlaneSearchRadius       = params.ICPOptions.PlaneSearchRadius;
    ICPOptions.MaxRoughness            = params.ICPOptions.MaxRoughness;
    ICPOptions.LogLevel                = params.ICPOptions.LogLevel;
    ICPOptions.Plot                    = params.ICPOptions.Plot;

    icptest.runICP(ICPOptions);


