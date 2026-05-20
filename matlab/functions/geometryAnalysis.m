function geometryAnalysis(configpath)
    logInfo(sprintf("Loading configuration file %s",configpath));
    tStage = tic;
    tTotal = tic;
    %Extracting JSON parameters and paths
    txt = fileread(configpath);
    params = jsondecode(txt);
    outputFolder = params.outputFolder;
    ICPtformsPath = strcat(outputFolder, params.preprocessing.afterICPpath, "TrafoPrmOrigin.txt");
    floorTformsPath = strcat(outputFolder, "floorTforms.aln");
    BeforeICPpath=strcat(outputFolder,params.preprocessing.beforeICPpath);
    random_seed = params.preprocessing.seed;
    rng(random_seed, "twister");

    %Load arrFiltered
    afterFilterPath = strcat(outputFolder, params.preprocessing.afterICPpath,"afterFilter.mat");
    load(afterFilterPath,"arrFiltered");
    
    %downsampling
    filtCloud=pccat(arrFiltered);
    filtCloudclean=pcdownsample(filtCloud,"gridAverage",params.geometry.downsampleGrid);

    %zNormsAnalysis - crop cloud
    [sortedPOI,p1Index,p2Index,p3Index] = zNormsAnalysis(filtCloudclean, 'false');
    pcTotalCake=cropCloud(filtCloudclean,[-inf,inf],[-inf,inf],[sortedPOI(p2Index+1), inf]);
    %pcTotalCake=cropCloud(filtCloudclean,[-inf,inf],[-inf,inf],[params.geometry.downsampleGrid, inf]);

    %Cropping step
    pcTotalCake=cropCloud(pcTotalCake,[-inf, 4.25],[-inf,inf],[-inf,inf]); %include in config

    %Export pcTotalCake
    pcwrite(pcTotalCake,"totalCake.ply");

    %Extract surface: make function with showplots optional
    pcTopFilled = isolateSurface(pcTotalCake);

    %Interpolate cloud + stats
    [F,X,Y]=interpolateCloud(pcTopFilled,params.geometry.interpolateGrid);
    
    %Stats
    Fclean=F(~isnan(F));
    % Extract valid points
    h = Fclean;  % column vector of all height values
    
    % --- Robust summary statistics ---
    mean_h     = mean(h);
    median_h   = median(h);
    std_h      = std(h);
    mad_h      = mad(h, 1);       % Median Absolute Deviation (scaled)
    skew_h     = skewness(h);
    kurt_h     = kurtosis(h);
    
    % Display numeric results
    fprintf('Height Statistics:\n');
    fprintf('  Mean:      %.4f\n', mean_h);
    fprintf('  Median:    %.4f\n', median_h);
    fprintf('  STD:       %.4f\n', std_h);
    fprintf('  MAD:       %.4f\n', mad_h);
    fprintf('  Skewness:  %.4f\n', skew_h);
    fprintf('  Kurtosis:  %.4f\n', kurt_h);

    %Rayleigh criterion 
%     n=109; %Determine dimensions of F
%     cutoff=25;
%     output = interestPeaks(F,X,Y,n,cutoff);
end