function [cropped] = crop(data)

% Crops data to nearest 0.1 m in x and y (to remove excess rims around
% sections).

% Input:
% data = trivariate data
%
% Output:
% cropped = cropped data

% Identify Limits
xmin=min(data(:,1));
ymin=min(data(:,2));

xrange=max(data(:,1)-min(data(:,1)));
xnew=floor(xrange/0.1)*0.1;

% Crop
if xrange >= xnew
    remx=find(data(:,1)>xmin & data(:,1)<(xmin+xnew));
    remy=find(data(:,2)>ymin & data(:,2)<(ymin+xnew));
    rem=[remx;remy];
    rem=sort(rem);
    idx=[false;diff(rem)<1];
    rem=rem(idx>0.5);
    cropped=data(rem,:);
elseif xrange < xnew
    cropped=data;
end;
   
end

