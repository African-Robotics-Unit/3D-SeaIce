function [section] = crop_section(data,xmin,xmax,ymin,ymax)

% crops section from trivariate data within x and y limits

remx=find(data(:,1)>xmin & data(:,1)<=xmax);
remy=find(data(:,2)>ymin & data(:,2)<=ymax);
rem=[remx;remy];
rem=sort(rem);
idx=[false;diff(rem)<1];
rem=rem(idx>0.5);
section=data(rem,:);

end

