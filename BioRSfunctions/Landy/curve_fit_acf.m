function [form1D,form2D] = curve_fit_acf(acf1D,lags,ACFnorm,scale,plot)
%
% Calculates the coefficident of determination between 
%
% Input:
% acf = averaged autocorrelation function from profiles in x and y
% lags = lag distances for acf
% ACFnorm = normalized, one-sided 2D autocorrelaton of surface
% scale = spacing of ACFnorm
% plot = type 'plot' for optional histograms of the 1D and 2D exponent
% values from the x-exp law
%
% Output:
% form1D & form 2D = look-up tables containing, in order:
    % expon = fraction of acf profiles closest to exponential form
    % gauss = fraction of acf profiles closest to gaussian form
    % n = average exponent of isotropic x-exp law through acf profiles
    % nstd = std dev of exponent of isotropic x-exp law through acf profiles
    % nrange = range of exponent of isotropic x-exp law through acf profiles
%

% Set up 2D ACF profiles
X=ones((size(ACFnorm,1)+1)/2,1)*(0:scale:scale*(((size(ACFnorm,1)+1)/2)-1));
Y=rot90(ones((size(ACFnorm,1)+1)/2,1)*(0:scale:scale*(((size(ACFnorm,1)+1)/2)-1)));

ACFquarter=ACFnorm(1:((size(ACFnorm,1)+1)/2),((size(ACFnorm,1)+1)/2):size(ACFnorm,1));
[~,RHO,Z] = cart2pol(X,Y,ACFquarter); % convert to polar coordinate system
Z=rot90(Z');

C=contours(X(1,:),Y(:,1),RHO,[max(max(X)) max(max(X))]);
C=C(:,2:size(C,2));

acf=zeros(size(C,2),size(ACFquarter,1)); % create profiles
for i=1:size(C,2)
    profile=improfile(X(1,:),Y(:,1),Z,[0 C(1,i)],[0 C(2,i)],size(ACFquarter,1));
    acf(i,:)=profile';
end;

% Increase sampling resolution of 2D ACF by factor of 10
acf2D=zeros(size(C,2),size(ACFquarter,1)*10);
for i=1:size(C,2)
    acf2D(i,:)=interp(acf(i,:),10);
end;

% Subsample (decimate) 1D and 2D ACFs by factor of 5
acf1D=acf1D(1:5:(floor(size(acf1D,1)/5)*5),:);
acf2D=acf2D(1:5:(floor(size(acf2D,1)/5)*5),:);

% ACF Models
exponential=fittype('exp(-x*a)');
gaussian=fittype('exp(-(x*a)^2)');
xexp=fittype('exp(-(a*x)^b)');

% Call ACF models for each 1D profile, calculate best fit & exponent n of
% x-power model
form=zeros(1,size(acf1D,1));
rsquare=zeros(1,size(acf1D,1));
n=zeros(1,size(acf1D,1));
for i=1:size(acf1D,1)
    k = 1;
    while (acf1D(i,k) > 1/exp(1))
        k = k + 1;
    end;
    cl = lags(k-1);
    lags_i=lags(lags<cl*3); % fit up to 3 correlation lengths
    acf1=acf1D(i,1:size(lags_i,2));
    [~,gof_e]=fit(lags_i',acf1',exponential,'start',1);
    [~,gof_g]=fit(lags_i',acf1',gaussian,'start',1);
    [res_x,gof_x]=fit(lags_i',acf1',xexp,'start',[1,1],'Lower',[-Inf,1],'Upper',[Inf,2]);
    if gof_e.rsquare > gof_g.rsquare
        form(:,i)=1;
    elseif gof_e.rsquare < gof_g.rsquare
        form(:,i)=2;
    elseif gof_e.rsquare == gof_g.rsquare
        form(:,i)=0;
    end;
    n(:,i)=res_x.b;
    rsquare(:,i)=gof_x.rsquare;
end;

% Export variables to object and display
expon=numel(find(form==1))/(numel(find(form==1))+numel(find(form==2)));
gauss=numel(find(form==2))/(numel(find(form==1))+numel(find(form==2)));
nmean=mean(n); nstd=std(n); nrange=max(n)-min(n); rsquare=mean(rsquare);
form1D=[expon gauss nmean nstd nrange rsquare];

disp('1D % Exponential vs Gaussian'); disp([expon gauss]);
disp('1D X-Exp ACF n mean std range r^2'); disp([nmean nstd nrange rsquare]);

% Plot 1D histogram of x-exp exponent
if nargin==5
    if strcmp(plot,'plot');
        subplot(1,2,1)
        [h,bc]=hist(n,20);
        h=h/sum(n);
        bar(bc,h)
        axis([1 2])
        xlabel('X-Exp ACF Exponent (n)')
        ylabel('Normalized Frequency')       
        title('Plot of X-Exp Exponent Values from 1D ACFs')
    end;
end;

% Call ACF models for each 2D profile, calculate best fit & exponent n of
% x-power model
form=zeros(1,size(acf2D,1));
rsquare=zeros(1,size(acf2D,1));
n=zeros(1,size(acf2D,1));
for j=1:size(acf2D,1)
    k = 1;
    while (acf2D(j,k) > 1/exp(1))
        k = k + 1;
    end;
    cl = lags(k-1);
    lags_j=lags(lags<cl*3); % fit up to 3 correlation lengths
    acf2=acf2D(j,1:size(lags_j,2));
    [~,gof_e]=fit(lags_j',acf2',exponential,'start',1);
    [~,gof_g]=fit(lags_j',acf2',gaussian,'start',1);
    [res_x,gof_x]=fit(lags_j',acf2',xexp,'start',[1,1],'Lower',[-Inf,1],'Upper',[Inf,2]);
    if gof_e.rsquare > gof_g.rsquare
        form(:,j)=1;
    elseif gof_e.rsquare < gof_g.rsquare
        form(:,j)=2;
    elseif gof_e.rsquare == gof_g.rsquare
        form(:,j)=0;
    end;
    n(:,j)=res_x.b;
    rsquare(:,j)=gof_x.rsquare;
end;

% Export variables to object and display
expon=numel(find(form==1))/(numel(find(form==1))+numel(find(form==2)));
gauss=numel(find(form==2))/(numel(find(form==1))+numel(find(form==2)));
nmean=mean(n); nstd=std(n); nrange=max(n)-min(n); rsquare=mean(rsquare);
form2D=[expon gauss nmean nstd nrange rsquare];

disp('2D % Exponential vs Gaussian'); disp([expon gauss]);
disp('2D X-Exp ACF n mean std range r^2'); disp([nmean nstd nrange rsquare]);

% Plot 1D histogram of x-exp exponent
if nargin==5
    if strcmp(plot,'plot');
        subplot(1,2,2)
        [h,bc]=hist(n,20);
        h=h/sum(n);
        bar(bc,h)
        axis([1 2])
        xlabel('X-Exp ACF Exponent (n)')
        ylabel('Normalized Frequency')       
        title('Plot of X-Exp Exponent Values from 2D ACFs')
    end;
end;

end