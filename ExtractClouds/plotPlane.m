function plotPlane(ptCloud,model)
pcDisplay(ptCloud);
hold on;
% Plot the fitted plane
plot(model);
title('Original Point Cloud with Fitted Plane');
legend('Point Cloud', 'Fitted Plane');
hold off
end