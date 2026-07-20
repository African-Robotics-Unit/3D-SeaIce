function [meta,parameters,std_dev] = sampling_extent(X,Y,R,scale,n)
%
% Calculates average roughness parameters in 1D and 2D for a surface split
% into smaller sections according to vector n.

n=n/scale;
parameters=zeros(length(n),19);
std_dev=zeros(length(n),19);
meta=cellstr(char(zeros(length(n),1)));

for i=1:length(n)
    fprintf(1, 'Now calculating roughness parameters for subsections with diameter: %g\n', n(i)*scale);
    subset=zeros((size(R,1)/n(i))^2,19);
    m=n(i);
    Xsub=X(1:m);
    Ysub=Y(1:m);
    count=1;
    
    for j=1:size(R,1)/n(i)
        for k=1:size(R,2)/n(i)
            fprintf(1, 'Subsection: %i\n', count);
            Rsub=R(((j*m)-m)+1:j*m,((k*m)-m)+1:k*m);
            [cl1,stdcl1,mincl1,maxcl1,~,acf1D,lags] = cl_profiles(Xsub,Rsub);
            [cl2,stdcl2,ecl2,ACFnorm] = cl_3DACF(Xsub,Ysub,Rsub);
            params=[cl1 stdcl1 mincl1 maxcl1 cl2 stdcl2 ecl2];
            subset(count,1:4)=params(1:4);
            subset(count,11:13)=params(5:7);
            %[form1D,form2D] = curve_fit_acf(acf1D,lags,ACFnorm,scale);
            %subset(count,5:10)=form1D;
            %subset(count,14:19)=form2D;
            count=count+1;
        end;
    end;
    
    meta(i,:)=cellstr([num2str(n(i)*scale) 'm subsections']);
    parameters(i,:)=mean(subset);
    std_dev(i,:)=std(subset);
    
end;
            
end

