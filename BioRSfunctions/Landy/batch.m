function [meta,parameters] = batch(folder,scale,detrend,cutoff)
% 
% Batch processes a series of raw LiDAR surface point clouds to calculate
% surface roughness parameters, including h, cl and ACF etc., using the
% SAME input parameters, i.e. scale, detrending technique & detrending
% cutoff.
%
% Input:
% folder = directory of folder containing raw surface point clouds
% e.g. ['C:\Users\LANDY\Desktop\PhD Data Analysis\Roughness
% Stats\Compilation for Paper 2\SERF2012_FFS'].
% scale = grid spacing (see 'roughness' function)
% detrend = detrending technique
% cutoff = cutoff wavelength or breakpoint spacing
%
% Output:
% meta = metadata for 'parameters', including name of surface
% parameters = table of parameters associated with metadata in each row,
% including 1D and 2D roughness parameters, of the form: [h cl1 stdcl1
% mincl1 maxcl1 expon1 gauss1 n1 nstd1 nrange1 rsquare1 cl2 stdcl2 ecl2
% expon2 gauss2 n2 nstd2 nrange2 rsquare2].

% Set up folder and file pattern
filepattern=fullfile(folder,'*.txt');
datfiles=dir(filepattern);

% Call roughness and ACF curve fit algorithms for each datum
meta=cellstr(char(zeros(length(datfiles),1)));
parameters=zeros(length(datfiles),20);
for k=1:length(datfiles)
    basefilename=datfiles(k).name;
    fullfilename=fullfile(folder,basefilename);
    fprintf(1, 'Now reading %s\n', fullfilename);
    data=dlmread(fullfilename);
    meta(k)=cellstr(fullfilename);
    fprintf(1, 'Now calculating roughness parameters for %s\n', basefilename);
    [~,params,acf1D,ACFnorm,lags]=roughness(data,scale,detrend,cutoff);
    parameters(k,1:5)=params(1:5);
    parameters(k,12:14)=params(6:8);
    fprintf(1, 'Now calculating ACF fit parameters for %s\n', basefilename);
    [form1D,form2D] = curve_fit_acf(acf1D,lags,ACFnorm,scale);
    parameters(k,6:11)=form1D;
    parameters(k,15:20)=form2D;
end;

end

