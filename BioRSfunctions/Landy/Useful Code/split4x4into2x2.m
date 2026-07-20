function [sec1,sec2,sec3,sec4] = split4x4into2x2(data)
%
% split 4 x 4 m section of trivariate data into 4 2 x 2 m sections for use
% with split 2x2
%
% Input:
% data = trivariate data
%
% Output:
% sec1-4 = trivariate subsections 1 to 4

xmin=min(data(:,1)); xmax=max(data(:,1));
ymin=min(data(:,2)); ymax=max(data(:,2));
xrange=xmax-xmin; yrange=ymax-ymin;
xhalf=xrange/2; yhalf=yrange/2;

remx1=find(data(:,1)>=xmin & data(:,1)<=(xmin+xhalf));
remy1=find(data(:,2)>=ymin & data(:,2)<=(ymin+yhalf));
rem1=sort([remx1;remy1]);
idx1=[false;diff(rem1)<1];
rem1=rem1(idx1>0.5);
sec1=data(rem1,:);

remx2=find(data(:,1)>=xmin & data(:,1)<=(xmin+xhalf));
remy2=find(data(:,2)>=(ymin+yhalf) & data(:,2)<=ymax);
rem2=sort([remx2;remy2]);
idx2=[false;diff(rem2)<1];
rem2=rem2(idx2>0.5);
sec2=data(rem2,:);

remx3=find(data(:,1)>=(xmin+xhalf) & data(:,1)<=xmax);
remy3=find(data(:,2)>=(ymin+yhalf) & data(:,2)<=ymax);
rem3=sort([remx3;remy3]);
idx3=[false;diff(rem3)<1];
rem3=rem3(idx3>0.5);
sec3=data(rem3,:);

remx4=find(data(:,1)>=(xmin+xhalf) & data(:,1)<=xmax);
remy4=find(data(:,2)>=ymin & data(:,2)<=(ymin+yhalf));
rem4=sort([remx4;remy4]);
idx4=[false;diff(rem4)<1];
rem4=rem4(idx4>0.5);
sec4=data(rem4,:);


end

