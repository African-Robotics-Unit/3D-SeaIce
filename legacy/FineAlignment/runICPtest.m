addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\GliraICP\"));
addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\"));
%addpath(genpath('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\LiDAR Extraction'));
%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\appDesign\ICPtestA\P3clean\','C:\Users\agori\Documents\MATLAB\MSc\appDesign\ICPtestA\P3ICP\');

%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\BeforeAlignment\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\AfterNew\');
%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\BeforeOutlierRemoved\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\AfterOutlierRemoved\');

%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\FilteredBeforeICP\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\FilteredAfterICP\')


%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\ObliqueBefore\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\ObliqueAfter\');

%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_Before\','C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_AfterTest1\');
%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_BeforeH2new\','C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\afterTesting\');

%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_BeforeNewTform\','C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_AfterNewTform\');


%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\adjustedRaw2026\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\adjustedRaw2026AFTER\');
icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\newAdjusted2026\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\newAdjusted2026AFTER\');

figure; icptest.plot('Color', 'by PC');
%ICPOptions.NoOfTransfParam          = 6; %Rigid body transform
%ICPOptions.NoOfTransfParam          = 3; %Rigid body transform
ICPOptions.NoOfTransfParam          = 6;
ICPOptions.UniformSamplingDistance  = 0.2;
ICPOptions.PlaneSearchRadius        = 0.1;
ICPOptions.MaxRoughness             = 0.02;
ICPOptions.LogLevel                 = 'debug';
ICPOptions.Plot                     = true;

icptest.runICP(ICPOptions);

saveICPclouds(icptest, 'C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\newAdjusted2026AFTER\',8);
%saveICPclouds(icptest, 'C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\FilteredAfterICP\',8);
%saveICPclouds(icptest, 'C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\P3processing\P3_AfterNewTform\',8);