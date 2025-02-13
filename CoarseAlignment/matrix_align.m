function [arrTforms] = matrix_align(x,y,a_len,b_len,c_len,d_len,e_len,f_len,g_len,h_len)

%A
theta=pi/4;
R_A = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
t_A = [(-1*(sqrt(2)/2)*(-(y + b_len) + (x/2)))-sind(45)*a_len;
      (-1*(sqrt(2)/2)*(x/2 + y + b_len))+y+sind(45)*a_len;
      0];
T_A = rigidtform3d(R_A, t_A);
%T_A = rigidtform3d(R_A, [0,0,0]);
%----------------------------------------------
%B - reference matrix
R_B = eye(3);
t_B = [-b_len,0,0];
%t_B= [x, y+b_len-f_len, 0];
T_B = rigidtform3d(R_B, t_B);
%----------------------------------------------
%C
theta=-pi/4;
R_C = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
t_C = [(-1*(sqrt(2)/2)*(x/2 + y + b_len))+x+sind(45)*c_len;
       (1*(sqrt(2)/2)*(-(y + b_len) + (x/2)))+y+sind(45)*c_len;
       0];

T_C = rigidtform3d(R_C, t_C);
%T_C = rigidtform3d(R_C, [0,0,0]);
%----------------------------------------------
%D
theta=-(pi/2);
R_D = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
t_D = [-(y+b_len)+x+d_len, (x/2)+y/2, 0];

%t_D=[y+b_len-h_len, -x/2+y/2, 0];
%T_D = rigidtform3d(R_D, t_D);
T_D = rigidtform3d(R_D, [0,0,0]);
%----------------------------------------------
%E
theta=pi + pi/4;
R_E = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];

t_E = [(1*((sqrt(2)/2)*(-(y + b_len) + (x/2))))+x+(e_len*sind(45));
    (1*(sqrt(2)/2)*(x/2 + y + b_len))-(e_len*sind(45));
    0];
T_E = rigidtform3d(R_E, t_E);
%T_E = rigidtform3d(R_E, [0,0,0]);
%----------------------------------------------
%F
theta=pi;
R_F = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
%t_F = [x, y+b_len-f_len, 0];
%t_F = [2*x, 0, 0];
t_F= [f_len,0,0];

T_F = rigidtform3d(R_F, t_F);
%T_F = rigidtform3d(R_F, [0,0,0]);
%----------------------------------------------
%G
theta=pi/2 + pi/4;
R_G = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];

t_G = [1*(sqrt(2)/2)*(x/2 + y + b_len)-g_len*sind(45);
     -1*((sqrt(2)/2)*(-(y + b_len) + (x/2)))-g_len*sind(45);
      0];
T_G = rigidtform3d(R_G, t_G);
%T_G = rigidtform3d(R_G, [0,0,0]);
%----------------------------------------------
%H
theta=pi/2;
R_H = [cos(theta),-sin(theta),  0;
        sin(theta),cos(theta),  0;
        0,0,1;];
t_H = [y+b_len-h_len, -x/2+y/2, 0];
%t_H = [-(y+b_len)+x+d_len, (x/2)+y/2, 0];
%T_H = rigidtform3d(R_H, t_H);
T_H = rigidtform3d(R_H, [0,0,0]);
%----------------------------------------------



arrTforms=[T_A,T_B,T_C,T_D,T_E,T_F,T_G,T_H];
%arrTforms=[T_A,T_B,T_C,T_H,T_E,T_F,T_G,T_D];
end