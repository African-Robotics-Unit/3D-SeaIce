function [pnts1_matrix, pnts2_matrix] = getPointPair(fig1, fig2)
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

%x = linspace(0,10,150);
%y = cos(5*x);
%fig = figure;
%plot(x,y,'Color',[0,0.7,0.9])
%title('2-D Line Plot')
%xlabel('x')
%ylabel('cos(5x)')
% fig1=figure;
% pcshow(pc1, "AxesVisibility","on")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title("pc1")
% fig2=figure;
% pcshow(pc2, "AxesVisibility","on")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title("pc2")


% Enable data cursor mode
datacursormode on
dcm_obj1 = datacursormode(fig1);
dcm_obj2 = datacursormode(fig2);
% Set update function
%set(dcm_obj,'UpdateFcn',@myupdatefcn)
% Wait while the user to click
%pnts1={};
%pnts2={};

pnts1_matrix = zeros(3,5);
pnts2_matrix = zeros(3,5);
%count1=0;
%count2=0;
count=0;
% for k = 1:5
    %disp('Click line to display a data tip, then press "Return"')
    disp('Select matching points in figure1 and figure2 then press enter')
    %pause 
    % Export cursor to workspace
    info_struct1 = getCursorInfo(dcm_obj1);
    info_struct2 = getCursorInfo(dcm_obj2);
    if ((isfield(info_struct1, 'Position')) && (isfield(info_struct2, 'Position')))
        count=count+1;
        disp('Clicked positioin of 1 is');
        disp(info_struct1.Position);

        disp('Clicked positioin of 2 is');
        disp(info_struct2.Position);
        
        pnts1_matrix(:,count)=info_struct1.Position;
        pnts2_matrix(:,count)=info_struct2.Position;
        
        %pnts1{count}=info_struct1.Position;
        %pnts2{count}=info_struct2.Position;
    else
        disp('Matching points not found please try again')
    end
  
%     if isfield(info_struct1, 'Position')
%         count1=count1+1;
%         disp('Clicked positioin of 1 is')
%         disp(info_struct1.Position)
%         pnts1{count1}=info_struct1.Position;
%     elseif isfield(info_struct2, 'Position')
%         count2=count2+1;
%         disp('Clicked positioin of 2 is')
%         disp(info_struct2.Position)
%         pnts2{count2}=info_struct2.Position;
 %   end
end