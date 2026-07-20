function iso_plot(R,ACFnorm,scale)

% Plots the 1/e contour of the surface height autocorrelation function and
% the directional dependence of the correlation length with respect to the
% x-axis

% Input:
% R = reconstructed surface
% ACFnorm = normalized, one-sided autocorrelation function of R
% scale = sample spacing

% Set up Grids
Xsize=size(R,2);
Ysize=size(R,1);

% Identify 1/e Contour
[C]=contours(ACFnorm,[1/exp(1) 1/exp(1)]);
Csize=size(C,2);
Xabs=Xsize*ones(1,Csize);
Yabs=Ysize*ones(1,Csize);
Cabs=[(C(1,:)-Xabs)',(C(2,:)-Yabs)']*scale;
Cabs(1,:)=[];

% Contour Vectors
vector=sqrt((C(1,:)-Xabs).^2+(C(2,:)-Yabs).^2);
vecm=vector'*scale;
ang=(rad2deg(atan2(C(2,:)-Yabs,C(1,:)-Xabs)))';
vecm(1,:)=[]; vecm(size(vecm,2),:)=[];
ang(1,:)=[]; ang(size(ang,2),:)=[];
vecang=sortrows(([ang vecm]),1);

% Plotting
figure;
subplot(1,2,1),plot(Cabs(:,1),Cabs(:,2))
axis([-1*Xsize*scale,Xsize*scale,-1*Ysize*scale,Ysize*scale])
xlabel('X Lags (m)')
ylabel('Y Lags (m)')
subplot(1,2,2),plot(vecang(:,1),vecang(:,2))
xlim([-180 180])
set(gca,'XTick',[-180 -90 0 90 180]);
xlabel('Angle from the X axis (deg)');
ylabel('Correlation length (m)');




end