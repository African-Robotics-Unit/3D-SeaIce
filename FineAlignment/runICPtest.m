addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\GliraICP"));
%addpath(genpath('C:\Users\agori\Documents\MATLAB\MSc\CRUISE-DATA\LiDAR Extraction'));
%icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\appDesign\ICPtestA\P3clean\','C:\Users\agori\Documents\MATLAB\MSc\appDesign\ICPtestA\P3ICP\');
icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\BeforeAlignment\','C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\AfterTrans\');
figure; icptest.plot('Color', 'by PC');

%ICPOptions.NoOfTransfParam          = 6; %Rigid body transform
ICPOptions.NoOfTransfParam          = 3; %Rigid body transform
ICPOptions.UniformSamplingDistance  = 0.2;
ICPOptions.PlaneSearchRadius        = 0.1;
ICPOptions.MaxRoughness             = 0.02;
ICPOptions.LogLevel                 = 'debug';
ICPOptions.Plot                     = true;

icptest.runICP(ICPOptions);