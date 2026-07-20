% calculate best fit parameters for various autocorrelation through
% experimental ACFs

% INPUT:
% x = lag distances of ACF
% y = correlation (p)

% OUTPUT:
% n = exponent of model fit (where appropriate)
% L - modelled correlation length
% rsquare = r-squared coeff of determination of best fit
% rmse = rmse of best fit to theoretical model

% exponential model
exponential=fittype('exp(-x*a)');
[res,gof]=fit(x,y,exponential,'start',1);
model=exp(-(0:(max(x)/(size(x,1)-1)):max(x))*res.a);
L=1/res.a;
rsquare=gof.rsquare;
rmse=gof.rmse;

% gaussian model
gauss=fittype('exp(-(x*a)^2)');
[res,gof]=fit(x,y,gauss,'start',1);
model=exp(-((0:(max(x)/(size(x,1)-1)):max(x))*res.a).^2);
L=1/res.a;
rsquare=gof.rsquare;
rmse=gof.rmse;

% isotropic x-power model
xpower=fittype('(1+(a*x^2))^-b');
[res,gof]=fit(x,y,xpower,'start',[1,1],'Lower',[-Inf,1]);
model=(1+(res.a*(0:(max(x)/(size(x,1)-1)):max(x)).^2)).^-(res.b);
n=res.b;
L=1/sqrt(res.a);
rsquare=gof.rsquare;
rmse=gof.rmse;


% isotropic x-exponential model
xexp=fittype('exp(-(a*x)^b)');
[res,gof]=fit(x,y,xexp,'start',[1,1],'Lower',[-Inf,1],'Upper',[Inf,2]);
model=exp(-(res.a*(0:(max(x)/(size(x,1)-1)):max(x))).^(res.b));
n=res.b;
L=1/res.a;
rsquare=gof.rsquare;
rmse=gof.rmse;





