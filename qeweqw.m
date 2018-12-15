
radius1 = 65;
radius2 = 80;
radius3 = 100;

v1_circle_dash = (2*pi*radius1)/10;
v2_circle_dash = (2*pi*radius2)/10;
v3_circle_dash = (2*pi*radius3)/10;

% v1_dash = 20;
% v2_dash = 20;
% v3_dash = 20;

%% agent 1
A1_circle = [0 1 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 -1];
 
B1_circle = [0,0;
             v1_circle_dash,-v1_circle_dash/radius1;
             0,1/radius1;
             0,0];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q1_circle = [1 0 0 0;
             0 1 0 0;
             0 0 1000 0;
             0 0 0 10];

R1_circle = [1 0;
             0 1];
  
K1_circle  = lqr(A1_circle,B1_circle,Q1_circle,R1_circle);
