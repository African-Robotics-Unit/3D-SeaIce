function arrCloudTrans = applyRStforms(arrLiDAR_rotated,arrTformsRigid)
    C_RL = [0, 0, -1, 0;
            -1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 0, 1];
    
    C_LR = C_RL';  
    arrTformsTotal = cell(1, 8);
    arrCloudTrans = repmat(pointCloud(zeros(0,3)), 8, 1);
    
    for i = 1:8
        arrTformsTotal{i} = rigidtform3d(C_RL * arrTformsRigid{i} * C_LR);
        arrCloudTrans(i)=pctransform(arrLiDAR_rotated(i), arrTformsTotal{i});
    end
end