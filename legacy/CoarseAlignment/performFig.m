function performFig(ptCloud)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% f1=figure;
% figure(f1);
% pcshow(ptCloud, "AxesVisibility","on")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title("pc1")
% datacursormode(f1, 'on');

x = linspace(0,10,150);
y = cos(5*x);
fig = figure;
plot(x,y,'Color',[0,0.7,0.9])
title('2-D Line Plot')
xlabel('x')
ylabel('cos(5x)')
% Enable data cursor mode
datacursormode on
dcm_obj = datacursormode(fig);
% Set update function
%set(dcm_obj,'UpdateFcn',@myupdatefcn)
% Wait while the user to click
for k = 1:5
    disp('Click line to display a data tip, then press "Return"')
    pause 
    % Export cursor to workspace
    info_struct = getCursorInfo(dcm_obj);
    if isfield(info_struct, 'Position')
        disp('Clicked positioin is')
        disp(info_struct.Position)
    end
end


end