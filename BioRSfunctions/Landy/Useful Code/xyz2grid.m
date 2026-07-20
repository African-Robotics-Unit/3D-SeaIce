% data = ordered trivariate list of grid cells (e.g. [x y z])



X=linspace(min(data(:,1)),max(data(:,1)),sqrt(size(data,1)));
Y=(linspace(min(data(:,2)),max(data(:,2)),sqrt(size(data,1))))';

grid=zeros(length(X),length(Y));
for i=1:size(Y)
    grid(:,i)=data((length(Y)*(i-1))+1:length(Y)*i,3);
end;

