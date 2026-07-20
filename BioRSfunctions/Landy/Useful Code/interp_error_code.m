
% cropping to small area of points 3 x 3 cm
remx=find(data(:,1)>=-1.61 & data(:,1)<=-1.58);
remy=find(data(:,2)>=0.99 & data(:,2)<=1.02);
rem=[remx;remy];
rem=sort(rem);
idx=[false;diff(rem)<1];
rem=rem(idx>0.5);
area=data(rem,:);

% cropping to small section of points 1 x 1 cm at centre of area
remx=find(data(:,1)>=-1.6 & data(:,1)<=-1.59);
remy=find(data(:,2)>=1 & data(:,2)<=1.01);
rem=[remx;remy];
rem=sort(rem);
idx=[false;diff(rem)<1];
rem=rem(idx>0.5);
section=data(rem,:);

% generate 2 mm grid of area and crop to section
xlin=linspace(-1.61,-1.58,16);
ylin=linspace(0.99,1.02,16);
[X,Y]=meshgrid(xlin,ylin);
F = TriScatteredInterp(area(:,1),area(:,2),area(:,3),'linear');
F=F(X,Y);
F=F(6:11,6:11);
X=X(6:11,6:11);
Y=Y(6:11,6:11);

% create x and y coordinates of residual vectors
X1=[section(:,1) section(:,1)];
X2=[section(:,2) section(:,2)];

% generate finer grid of section at 0.0001 m resolution
xlin=linspace(-1.6,-1.59,(0.01/0.0001)+1);
ylin=linspace(1,1.01,(0.01/0.0001)+1);
[X_2,Y_2]=meshgrid(xlin,ylin);
ZI=interp2(X,Y,F,X_2,Y_2);

% round variables to nearest 0.0001m
section=(round(section*10000))/10000;
xlin=(round(xlin*10000))/10000;
ylin=(round(ylin*10000))/10000;

% identify point locations on interpolated surface
output=[section(:,1:2) zeros(size(section,1),1)];
for i = 1:size(section,1)
    remx=find(xlin' == section(i,1));
    remy=find(ylin' == section(i,2));
    output(i,3)=ZI(remy,remx);
end;

% create z coordinates of residual vectors
X3=[section(:,3) output(:,3)];

% plot
figure
mesh(X,Y,F,'EdgeColor',[0 0 0],'FaceAlpha',0)
hold on
plot3(X1',X2',X3','-',section(:,1),section(:,2),section(:,3),'.','Color','b','MarkerSize',18)
hold off
%axis([-1.6 -1.59 1 1.01])

% copy figure to illustrator:
% file/export setup >>> custom renderer: painters