function saveClouds(arrClouds,exportPath, fname)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
arrLetters=["A", "B", "C", "D", "E", "F", "G", "H"];

% Check if the directory exists
if ~exist(exportPath, 'dir')
    % If the directory does not exist, create it
    mkdir(exportPath);
end

for k=1:length(arrClouds)
   % curFname = exportPath + fname + "-" + arrLetters{k} + ".ply";
    curFname= strcat(exportPath,fname,arrLetters{k},".ply");
    strings=curFname;
    pcwrite(arrClouds(k), curFname);
end