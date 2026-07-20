function [R] = OLSR(F,scale,breaks)

% Detrends a regularly gridded, square, two-dimensional surface F using
% piecewise ordinary least-squares regression (OLSR) planes.

% Input:
% F = regularly gridded surface model (DEM)
% scale = grid spacing in meters
% breaks = length in meters of square side of one OLSR plane, default = no
% breaks (Note: break length rounds to nearest factor of one dimension of F)

% Output:
% R = gridded surface model reconstructed after removal of piecewise
% regression planes

% Usage:
% [R] = OLSR(F,0.002,0.1)
% [R] = OLSR(F,0.02,[])

% Identify number of points within each break
breaks = breaks/scale;

% Set up grid of regression planes and run OLSR algroithm
R=zeros(size(F,1),size(F,2));
remx=rem(size(F,2),breaks);
remy=rem(size(F,1),breaks);

for i=1:floor(size(F,1)/breaks)
    for j=1:floor(size(F,2)/breaks)
        if i~=max(i) || j~=max(j)
            Fsub=F(((i*breaks)-breaks)+1:i*breaks,((j*breaks)-breaks)+1:j*breaks);
            [X,Y] = meshgrid(1:size(Fsub,2),1:size(Fsub,1)); % grid
            coeff = [X(:) Y(:) ones(size(X(:)))]\Fsub(:); % coeffs of OLS plane
            Fp = coeff(1)*X + coeff(2)*Y + coeff(3);
            Rsub = Fsub-Fp; % removal of plane from data
            R(((i*breaks)-breaks)+1:i*breaks,((j*breaks)-breaks)+1:j*breaks)=(Rsub);
        elseif i==max(i) && j~=max(j)
            Fsub=F(((i*breaks)-breaks)+1:(i*breaks)+remy,((j*breaks)-breaks)+1:j*breaks);
            [X,Y] = meshgrid(1:size(Fsub,2),1:size(Fsub,1)); 
            coeff = [X(:) Y(:) ones(size(X(:)))]\Fsub(:); 
            Fp = coeff(1)*X + coeff(2)*Y + coeff(3);
            Rsub = Fsub-Fp; 
            R(((i*breaks)-breaks)+1:(i*breaks)+remy,((j*breaks)-breaks)+1:j*breaks)=(Rsub);
        elseif i~=max(i) && j==max(j)
            Fsub=F(((i*breaks)-breaks)+1:i*breaks,((j*breaks)-breaks)+1:(j*breaks)+remx);
            [X,Y] = meshgrid(1:size(Fsub,2),1:size(Fsub,1)); 
            coeff = [X(:) Y(:) ones(size(X(:)))]\Fsub(:); 
            Fp = coeff(1)*X + coeff(2)*Y + coeff(3);
            Rsub = Fsub-Fp; 
            R(((i*breaks)-breaks)+1:i*breaks,((j*breaks)-breaks)+1:(j*breaks)+remx)=(Rsub);
        elseif i==max(i) && j==max(j)
            Fsub=F(((i*breaks)-breaks)+1:(i*breaks)+remy,((j*breaks)-breaks)+1:(j*breaks)+remx);
            [X,Y] = meshgrid(1:size(Fsub,2),1:size(Fsub,1)); 
            coeff = [X(:) Y(:) ones(size(X(:)))]\Fsub(:);
            Fp = coeff(1)*X + coeff(2)*Y + coeff(3);
            Rsub = Fsub-Fp;
            R(((i*breaks)-breaks)+1:(i*breaks)+remy,((j*breaks)-breaks)+1:(j*breaks)+remx)=(Rsub);
        end;
    end;
end






end

