function [grid_h,grid_cl2,grid_stdcl2,grid_ecl2,grid_n] = spatial_process(data,xmin,ymin)

% Calculates roughness parameter grids for 1x1 m surface sections
% starting at xmin and ymin, for trivariate data

xmax=xmin+round(max(data(:,1))-xmin);
ymax=ymin+round(max(data(:,2))-ymin);

grid_h=zeros(xmax-xmin,ymax-ymin);
grid_cl2=zeros(xmax-xmin,ymax-ymin);
grid_stdcl2=zeros(xmax-xmin,ymax-ymin);
grid_ecl2=zeros(xmax-xmin,ymax-ymin);
grid_n=zeros(xmax-xmin,ymax-ymin);

count=1;
for i=1:size(grid_h,1)
    for j=1:size(grid_h,2)
        [section]=crop_section(data,xmin+(i-1),xmin+i,ymin+(j-1),ymin+j);
        fprintf(1, 'Now calculating roughness parameters for section'); disp(count);
        [~,params,acf1D,ACFnorm,lags]=roughness(section,0.002,2,0.5);
        fprintf(1, 'Now calculating curve fit parameters for section'); disp(count);
        [~,form2D]=curve_fit_acf(acf1D,lags,ACFnorm,0.002);
        grid_h(i,j)=params(1);
        grid_cl2(i,j)=params(6);
        grid_stdcl2(i,j)=params(7);
        grid_ecl2(i,j)=params(8);
        grid_n(i,j)=form2D(3);
        count=count+1;
    end;
end;

rot90(rot90(rot90(grid_h)));
rot90(rot90(rot90(grid_cl2)));
rot90(rot90(rot90(grid_stdcl2)));
rot90(rot90(rot90(grid_ecl2)));
rot90(rot90(rot90(grid_n)));

end

