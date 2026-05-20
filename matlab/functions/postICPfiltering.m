function postICPfiltering(configpath)
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


    %Load point clouds beforeICP
    matFilename = strcat(BeforeICPpath, "beforeICP.mat");
    logInfo(sprintf("Loading point clouds from %s",matFilename));
    load(matFilename,"arrCoarse");

    %Getting transforms and point cloud array
    logInfo(sprintf("Extracting ICP transforms from %s",ICPtformsPath));
    ICPtforms = readTformFile(ICPtformsPath);
    logInfo(sprintf("Extracting floor transforms from %s",floorTformsPath));
    floorTforms=extractALNtransformRigid(floorTformsPath);
    
    logInfo("Applying ICP transforms and floor adjustment");
    arrAfterICP=arrCoarse;
    arrAfterICProtate=arrCoarse;
    %apply ICPtforms
    for k=1:8
        arrAfterICP(k)=pctransform(arrCoarse(k),ICPtforms(k));
        %correct entire with floorA
        arrAfterICProtate(k)=pctransform(arrAfterICP(k), rigidtform3d(floorTforms{1}));
    end
    logInfo(sprintf("ICP transforms and floor adjustment complete (+%.3fs)", toc(tStage)));


    intMin = params.intensityFilter.intensityMin;
    intMax = params.intensityFilter.intensityMax;
    logInfo(sprintf("Performing intensity filter, including only intensities in range [%d, %d]",intMin, intMax));
    %Applying intensity filtering
    arrIntensityFilt=arrAfterICProtate;
    for k=1:8
        arrIntensityFilt(k)=filterIntensityCloud(arrAfterICProtate(k),intMin,intMax);
    end
    logInfo(sprintf("Intensity filtering complete (+%.3fs)", toc(tStage)));

    if params.overlapFilter
        logInfo("Performing overlap filter")
        %Apply overlap filter
        [arrFiltered,arrOutliers] = overlapFilterOptimal(arrIntensityFilt);       
        logInfo(sprintf("Overlap filter complete (+%.3fs)", toc(tStage)));
    end
    
    totalFilt=pccat(arrFiltered);
    filterTest=totalFilt;
    outlierTest=pccat(arrOutliers);

    filterCol = pointCloud(filterTest.Location, 'Color', repmat(uint8([0, 255, 0]), size(filterTest.Location, 1), 1));
    outlierCol=pointCloud(outlierTest.Location, 'Color', repmat(uint8([0, 0, 255]), size(outlierTest.Location, 1), 1));
    pcDisplay(filterCol);
    hold on
    pcDisplay(outlierCol);
    hold off
    afterFilterPath = strcat(outputFolder, params.preprocessing.afterICPpath,"afterFilter.mat");
    save(afterFilterPath,"arrIntensityFilt","arrOutliers","arrFiltered","arrAfterICProtate");
    totalTime = toc(tTotal);
    hours = floor(totalTime / 3600);
    minutes = floor(mod(totalTime, 3600) / 60);
    seconds = mod(totalTime, 60);
    
    logInfo(sprintf("Pipeline completed successfully. Total runtime = %02dh:%02dm:%05.2fs",hours, minutes, seconds));
end