function showPair(pc1,pc2,graphName)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure;
pcshowpair(pc1,pc2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(graphName)
end