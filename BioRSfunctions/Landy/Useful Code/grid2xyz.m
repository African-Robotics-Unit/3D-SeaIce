% data = surface grid (e.g. F)
% X = scalar of x values on grid
% Y = scalar of y values on grid



trivariate=zeros(size(data,1)*size(data,2),3);
count=1;
for i=1:size(data,1)
    for j=1:size(data,2)
        trivariate(count,1)=X(i);
        trivariate(count,2)=Y(j);
        trivariate(count,3)=data(i,j);
        count=count+1;
    end;
end;

