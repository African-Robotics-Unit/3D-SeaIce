function arrLiDAR_ordered = preprocessClouds(configpath)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    %txt = fileread("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\matlab\config\p3_config.json");
    logInfo(sprintf("Loading preprocessing configuration file %s",configpath));
    tStage = tic;
    tTotal = tic;
    txt = fileread(configpath);
    params = jsondecode(txt);
    outputFolder = params.outputFolder;
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    pathname = params.preprocessing.pathname;
    gridStep = params.preprocessing.gridStep;
    x_roi = decodeJsonROI(params.preprocessing.roi.x);
    y_roi = decodeJsonROI(params.preprocessing.roi.y);
    z_roi = decodeJsonROI(params.preprocessing.roi.z);
    
    post_x_roi = decodeJsonROI(params.preprocessing.postFloorROI.x);
    post_y_roi = decodeJsonROI(params.preprocessing.postFloorROI.y);
    post_z_roi = decodeJsonROI(params.preprocessing.postFloorROI.z);
    postROI=[post_x_roi, post_y_roi, post_z_roi];

    nclouds = params.preprocessing.nclouds;
    random_seed = params.preprocessing.seed;
    startCloud = params.preprocessing.startCloud;
    scanOrder = params.preprocessing.scanOrder;
    save_arrLiDAR = params.preprocessing.save_arrLiDAR;
    roi = [x_roi, y_roi, z_roi];
    rng(random_seed, "twister");
    
    logInfo(sprintf("Configuration loaded successfully (+%.3fs)", toc(tStage)));
    
    
    logInfo(sprintf("Reading ROS bag files from %s",pathname));
    tStage = tic;
    
    [all_clouds, fnames] = rosGetRaw(pathname);
    %[minsz, testout] = getMinSz(all_clouds, fnames);
    
    logInfo(sprintf("ROS bag parsing complete (+%.3fs)", toc(tStage)));
    
    
    msg = sprintf([ ...
        'Performing cropMergeDownsample:\n' ...
        '    ROI crop:\n' ...
        '        X = %s\n' ...
        '        Y = %s\n' ...
        '        Z = %s\n' ...
        '    Voxel grid size = %.3f m\n' ...
        '    Frame merge count = %d\n' ...
        '    Frame range = [%d, %d]'], ...
        mat2str(x_roi), ...
        mat2str(y_roi), ...
        mat2str(z_roi), ...
        gridStep, ...
        nclouds, ...
        startCloud, ...
        startCloud + nclouds - 1);
    
    logInfo(msg);
    tStage = tic;
    
    arrLiDAR = cropMergeDownsample(roi, nclouds, gridStep, all_clouds, startCloud);
    
    logInfo(sprintf("Point cloud preprocessing complete (+%.3fs)", toc(tStage)));
    
    
    logInfo(sprintf("Reordering point clouds using fieldnotes %s", scanOrder));
    tStage = tic;
    
    arrLiDAR_ordered = reorderPointCloud(arrLiDAR, scanOrder);
    
    logInfo(sprintf("Point cloud reordering complete (+%.3fs)", toc(tStage)));
    
    if save_arrLiDAR
        arrLiDARfolder=strcat(outputFolder,params.preprocessing.save_arrLiDARfolder);
        saveClouds(arrLiDAR_ordered,arrLiDARfolder,params.preprocessing.save_arrLiDARname);
    end

    alnFilename = strcat(outputFolder, "floorTforms.aln");
    logInfo(sprintf("Performing floor removal saving to %s", alnFilename));
    tStage = tic;
    %Floor removal
    arrLiDAR=arrLiDAR_ordered;
    croppedClouds=arrLiDAR;
    croppedRotated=arrLiDAR;
    floorTforms = cell(8,1);
    %remove floor points for each arrCloudTrans
    for k=1:8
        [floorAlign,tform]=alignFloorTform(arrLiDAR_ordered(k));
        floorTforms{k}=tform;
        invTform = invert(tform);
        %crop floor out
        [sortedPOI,arrIndex,palletIndex,floorIndex,surfIndex] = normsAnalysis(floorAlign);
        croppedClouds(k)=cropCloud(floorAlign,[-inf,inf],[-inf,inf],[sortedPOI(floorIndex+1)+0.02,inf]);
        croppedRotated(k)=pctransform(croppedClouds(k),invTform);
    end
    
    writeALNtransformRigid(alnFilename,floorTforms);
    
    logInfo(sprintf("Floor removal completed and saved (+%.3fs)", toc(tStage)));
   

    msg = sprintf([ ...
        'Performing post floor removal crop:\n' ...
        '    ROI crop:\n' ...
        '        X = %s\n' ...
        '        Y = %s\n' ...
        '        Z = %s\n' ], ...
        mat2str(post_x_roi), ...
        mat2str(post_y_roi), ...
        mat2str(post_z_roi));
    
    logInfo(msg);
    tStage = tic;
    arrLiDAR_rotated=cropClouds(croppedRotated,postROI);
    logInfo(sprintf("Post floor crop complete (+%.3fs)", toc(tStage)));

    
    logInfo(sprintf("Extracting coarse alignment from %s", params.preprocessing.coarseALN));
    tStage = tic;
    arrTformsRigid=extractALNtransformRigid(params.preprocessing.coarseALN);
    arrCoarse = applyRStforms(arrLiDAR_rotated,arrTformsRigid);
    BeforeICPpath=strcat(outputFolder,params.preprocessing.beforeICPpath);
    
    logInfo(sprintf("Saving coarse aligned clouds to  %s", BeforeICPpath));
    saveClouds(arrCoarse,BeforeICPpath,params.preprocessing.fname);
    matFilename = strcat(BeforeICPpath, "beforeICP.mat");
    save(matFilename,"arrCoarse");
    logInfo(sprintf("Coarse alignment completed and saved (+%.3fs)", toc(tStage)));
    
    totalTime = toc(tTotal);
    hours = floor(totalTime / 3600);
    minutes = floor(mod(totalTime, 3600) / 60);
    seconds = mod(totalTime, 60);
    
    logInfo(sprintf("Pipeline completed successfully. Total runtime = %02dh:%02dm:%05.2fs",hours, minutes, seconds));
end