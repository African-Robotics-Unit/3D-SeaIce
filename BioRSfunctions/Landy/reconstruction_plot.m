function reconstruction_plot(F,R,scale,cutoff)

% Plots surface elevation models, in their original form and after
% reconstruction following detrending.

% Input:
% F = Original surface model F(X,Y)
% R = Reconstructed surface model R(X,Y)
% scale = Sample spacing
% cutoff = cutoff wavelength or break distance in detrending algorithm

% Create X and Y Vectors
%X=0:scale*(size(F,1)/(size(F,1)-1)):scale*size(F,1); %old
%Y=0:scale*(size(F,2)/(size(F,2)-1)):scale*size(F,2);
X=0:scale:scale*(size(F,1)-1);
Y=0:scale:scale*(size(F,2)-1);

% 2D Plots
figure;
subplot(1,2,1),imagesc(X,Y,F)
colorbar('location','EastOutside')
xlabel('Distance (m)')
ylabel('Distance (m)')
title('Original Surface')
subplot(1,2,2),imagesc(X,Y,R)
colorbar('location','EastOutside')
xlabel('Distance (m)')
ylabel('Distance (m)')
title(['Reconstructed Surface with ' num2str(cutoff) 'm cutoff'])

% 1D Profiles
figure;
subplot(2,1,1),plot(X,F(round(size(F,1)/2),:))
xlabel('Distance (m)')
ylabel('Height Below Scanner (m)')
title('Profile through Centre of Original Surface');
subplot(2,1,2),plot(X,R(round(size(R,1)/2),:))
xlabel('Distance (m)')
ylabel('Height (m)')
title(['Profile through Reconstructed Surface with ' num2str(cutoff) 'm cutoff']);

end