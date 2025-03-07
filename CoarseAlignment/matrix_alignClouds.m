function [arrTforms,arrRotated,arrShifted] = matrix_alignClouds(arrLiDAR,x,y,a_len,b_len,c_len,d_len,e_len,f_len,g_len,h_len)
%arrTforms: final rotation and translation
%arrRotated: shifted + applied rotation
%arrShifted: raw shifted data w/floor removed
arrDist=[a_len,b_len,c_len,d_len,e_len,f_len,g_len,h_len];
croppedClouds=arrLiDAR;
arrShifted=arrLiDAR;
for k=1:length(arrDist)
    %align floor
    floorAlign=alignFloor(arrLiDAR(k));
    %crop floor out
    [sortedPOI,arrIndex,palletIndex,floorIndex,surfIndex] = normsAnalysis(floorAlign);
    croppedClouds(k)=cropCloud(floorAlign,[-inf,inf],[-inf,inf],[sortedPOI(floorIndex+1)+0.02,inf]);
    %shift by x by arrDist +0.2
    xshift=rigidtform3d(eye(3), [-1*(arrDist(k)+0.2),0,0]);
    arrShifted(k)=pctransform(croppedClouds(k),xshift);
end

%generate transforms (matrix_align)
tformD=[x/2,y/2,0];
tformF=[x,0,0];
tformB=[0,0,0];
tformH=[x/2,-y/2,0];
tformA=[0,-y/2,0];
tformC=[0,y/2,0];
tformE=[x,y/2,0];
tformG=[x,-y/2,0];
arrTrans=[tformA;tformB;tformC;tformD;tformE;tformF;tformG;tformH];
arrTforms=[];
arrRotated=arrShifted;
arrTheta=[pi/4,0,-pi/4,-pi/2,pi + pi/4,pi,pi/2 + pi/4,pi/2];
for k=1:length(arrDist)
    theta=arrTheta(k);
    R = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
    curTform=rigidtform3d(R,[arrTrans(k,1),arrTrans(k,2),arrTrans(k,3)]);
    arrRotated(k)=pctransform(arrShifted(k),curTform);
    arrTforms=[arrTforms,curTform];
end
end