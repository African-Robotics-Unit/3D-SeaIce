addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\GliraICP\"));
addpath(genpath("C:\Users\agori\Documents\MATLAB\3D-Sea-Ice\FineAlignment\"));

icptest=runGliraICP('C:\Users\agori\Documents\MATLAB\MSc\P3processing-clean\Pancake3_Before\','C:\Users\agori\Documents\MATLAB\MSc\P3processing-clean\Pancake3_After\');

figure; icptest.plot('Color', 'by PC');
ICPOptions.NoOfTransfParam          = 6;
ICPOptions.UniformSamplingDistance  = 0.2;
ICPOptions.PlaneSearchRadius        = 0.1;
ICPOptions.MaxRoughness             = 0.02;
ICPOptions.LogLevel                 = 'debug';
ICPOptions.Plot                     = true;

icptest.runICP(ICPOptions);

saveICPclouds(icptest, 'C:\Users\agori\Documents\MATLAB\MSc\P3processing-clean\Pancake3_After\',8);
