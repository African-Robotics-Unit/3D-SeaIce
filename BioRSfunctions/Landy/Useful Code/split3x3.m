function [sec1,sec2,sec3,sec4,sec5,sec6,sec7,sec8,sec9] = split3x3(data)
%
% split 3 x 3 m section of trivariate data into 9 1 x 1 m sections
%
% Input:
% data = trivariate data
%
% Output:
% sec1-9 = trivariate subsections 1 to 9

xmin=min(data(:,1)); xmax=max(data(:,1));
ymin=min(data(:,2)); ymax=max(data(:,2));
xrange=xmax-xmin; yrange=ymax-ymin;
xthird=xrange/3; ythird=yrange/3;
xtwothird=xthird*2; ytwothird=ythird*2;

remx1=find(data(:,1)>=xmin & data(:,1)<=(xmin+xthird));
remy1=find(data(:,2)>=ymin & data(:,2)<=(ymin+ythird));
rem1=sort([remx1;remy1]);
idx1=[false;diff(rem1)<1];
rem1=rem1(idx1>0.5);
sec1=data(rem1,:);

remx2=find(data(:,1)>=(xmin+xthird) & data(:,1)<=(xmin+xtwothird));
remy2=find(data(:,2)>=ymin & data(:,2)<=(ymin+ythird));
rem2=sort([remx2;remy2]);
idx2=[false;diff(rem2)<1];
rem2=rem2(idx2>0.5);
sec2=data(rem2,:);

remx3=find(data(:,1)>=(xmin+xtwothird) & data(:,1)<=xmax);
remy3=find(data(:,2)>=ymin & data(:,2)<=(ymin+ythird));
rem3=sort([remx3;remy3]);
idx3=[false;diff(rem3)<1];
rem3=rem3(idx3>0.5);
sec3=data(rem3,:);

remx4=find(data(:,1)>=xmin & data(:,1)<=(xmin+xthird));
remy4=find(data(:,2)>=(ymin+ythird) & data(:,2)<=(ymin+ytwothird));
rem4=sort([remx4;remy4]);
idx4=[false;diff(rem4)<1];
rem4=rem4(idx4>0.5);
sec4=data(rem4,:);

remx5=find(data(:,1)>=(xmin+xthird) & data(:,1)<=(xmin+xtwothird));
remy5=find(data(:,2)>=(ymin+ythird) & data(:,2)<=(ymin+ytwothird));
rem5=sort([remx5;remy5]);
idx5=[false;diff(rem5)<1];
rem5=rem5(idx5>0.5);
sec5=data(rem5,:);

remx6=find(data(:,1)>=(xmin+xtwothird) & data(:,1)<=xmax);
remy6=find(data(:,2)>=(ymin+ythird) & data(:,2)<=(ymin+ytwothird));
rem6=sort([remx6;remy6]);
idx6=[false;diff(rem6)<1];
rem6=rem6(idx6>0.5);
sec6=data(rem6,:);

remx7=find(data(:,1)>=xmin & data(:,1)<=(xmin+xthird));
remy7=find(data(:,2)>=(ymin+ytwothird) & data(:,2)<=ymax);
rem7=sort([remx7;remy7]);
idx7=[false;diff(rem7)<1];
rem7=rem7(idx7>0.5);
sec7=data(rem7,:);

remx8=find(data(:,1)>=(xmin+xthird) & data(:,1)<=(xmin+xtwothird));
remy8=find(data(:,2)>=(ymin+ytwothird) & data(:,2)<=ymax);
rem8=sort([remx8;remy8]);
idx8=[false;diff(rem8)<1];
rem8=rem8(idx8>0.5);
sec8=data(rem8,:);

remx9=find(data(:,1)>=(xmin+xtwothird) & data(:,1)<=xmax);
remy9=find(data(:,2)>=(ymin+ytwothird) & data(:,2)<=ymax);
rem9=sort([remx9;remy9]);
idx9=[false;diff(rem9)<1];
rem9=rem9(idx9>0.5);
sec9=data(rem9,:);

end

