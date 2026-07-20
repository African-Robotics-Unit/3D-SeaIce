function acf_plot(acf,lags,scale,ACFnorm)
%
% Plots the averaged autocovariance functions calculated from profiles in
% x and y directions across the surface and from 2D cross correlation.
%
% Input:
% acf = averaged autocorrelation function from profiles in x and y
% lags = lag distances for acf
% ACFnorm = normalized, one-sided 2D autocorrelaton of surface
% scale = spacing of ACFnorm
%

format long

% For 2D ACF
X=ones((size(ACFnorm,1)+1)/2,1)*(0:scale:scale*(((size(ACFnorm,1)+1)/2)-1));
Y=rot90(ones((size(ACFnorm,1)+1)/2,1)*(0:scale:scale*(((size(ACFnorm,1)+1)/2)-1)));

ACFplot=ACFnorm(1:((size(ACFnorm,1)+1)/2),((size(ACFnorm,1)+1)/2):size(ACFnorm,1));
[THETA,RHO,Z] = cart2pol(X,Y,ACFplot);
cat_lags=(0:scale:scale*size(ACFplot,1));

acf2D=zeros(1,size(ACFplot,1));
for i=1:size(ACFplot,1)
    rem=find(RHO>=cat_lags(:,i) & RHO<=cat_lags(:,i+1));
    if i==1
        if isempty(rem)
            Zmean=1;
        else
            Zmean=mean(Z(rem));
        end;
    elseif i>1
        Zmean=mean(Z(rem));
    end;
    acf2D(i)=Zmean;
end;

% Plotting
figure;
subplot(1,2,1),plot(lags,acf)
xlabel('Lag Distance (m)')
ylabel('Autocorrelation (normalized)')            
title('Plot of the Average Normalized ACF from 1D Profiles Across the Surface')

%subplot(1,2,2),plot(cat_lags(2:size(cat_lags,2)),acf2D)
subplot(1,2,2),plot(cat_lags,[1 acf2D])
xlim([0 max(cat_lags)])
xlabel('Lag Distance (m)')
ylabel('Autocorrelation (normalized)')            
title('Plot of the Average Normalized ACF from 2D Cross-Correlation Across the Surface')
        
end