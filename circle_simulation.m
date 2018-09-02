clear all;
close all;
total_time = 20;
radius1 = 4;
radius2 = 2;
v1_dash = (2*pi*radius1)/total_time;
v2_dash = (2*pi*radius2)/total_time;

A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];
 
B = [0,0;
     v1_dash,0;
     0,0;
     0,v2_dash];
 
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 
D = 0;

% u(1) = si_1_dot;
% u(2) = si_2_dot;
% u(3) = v1_dot;
% u(4) = v2_dot;

states = {'d1' 'd1_dot' 'd2' 'd2_dot'};
inputs = {'u1' 'u2'};
outputs = {'pos_error1'; 'vel_error1';'pos_error2'; 'vel_error2'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q = [10 0 0 0;
     0 1 0 0;
     0 0 10 0;
     0 0 0 1];

R = [1 0;
     0 1];
  
K  = lqr(A,B,Q,R)  ;

% Ac = (A-B*K);
% Bc = B;
% Cc = C;
% Dc = D;

dt = 0.01;
T = 0:dt:total_time;

W1 = [4,0,-pi/2,0,0];           % x,y,si,centre_x_centre_y
W2 = [0,2,0,0,0];
x1 = W1(1);
y1 = W1(2);
x2 = W2(1);
y2 = W2(2);

figure
title('Step Response of Error in virtual time using LQR Control')

qwerty = size(T);

si_1 = W1(3);
if si_1 > 2*1.57
    si_1 = -(2*pi - si_1);
end
if si_1 < -2*1.57
    si_1 = (2*pi + si_1);
end
x_coordinate_1 = W1(1);
y_coordinate_1 = W1(2);
si_2 = W2(3);
if si_2 > 2*1.57
    si_2 = -(2*pi - si_2);
end
if si_2 < -2*1.57
    si_2 = (2*pi + si_2);
end
x_coordinate_2 = W2(1);
y_coordinate_2 = W2(2);

u = [0;0];

for c = 1:qwerty(2)
t = T(c); 

R1 = sqrt((x_coordinate_1 - W1(4))^2 + (y_coordinate_1 - W1(5))^2);
R2 = sqrt((x_coordinate_2 - W2(4))^2 + (y_coordinate_2 - W2(5))^2);

theta_1 =atan2(y_coordinate_1,x_coordinate_1);
theta_2 =atan2(y_coordinate_2,x_coordinate_2);

alpha1 = theta_1 - 1.57;
alpha2 = theta_2 - 1.57;
% if theta1 >= 1.57
%     alpha1 = theta1 - 1.57;
% end
% if theta2 >= 1.57
%     alpha2 = theta2 - 1.57;
% end
% if theta1 <= -1.57
%     alpha1 = 

%alpha = abs(theta_1 -theta_2);
%d1 = R1*sin(theta_1 - si_1);
d1 = R1-radius1;
Vd1 = v1_dash*sin(si_1-alpha1);
%d2 = R2*sin(theta_2 - si_2);
d2 = R2-radius2;
Vd2 = v2_dash*sin(si_2-alpha2);

X = [d1 ; Vd1 ; d2 ; Vd2];

%X_dot = Ac*X;
u = -K*X;
v = u + [(v1_dash)/radius1 ; (v2_dash)/radius2];

%updating states
si_1 = si_1 + v(1)*dt;
if si_1 > 2*1.57
    si_1 = -(2*pi - si_1);
end
if si_1 < -2*1.57
    si_1 = (2*pi + si_1);
end
x_coordinate_1 = x_coordinate_1 + v1_dash*cos(si_1)*dt;
y_coordinate_1 = y_coordinate_1 + v1_dash*sin(si_1)*dt;
si_2 = si_2 + v(2)*dt;
if si_2 > 2*1.57
    si_2 = -(2*pi - si_2);
end
if si_2 < -2*1.57
    si_2 = (2*pi + si_2);
end
x_coordinate_2 = x_coordinate_2 + v2_dash*cos(si_2)*dt;
y_coordinate_2 = y_coordinate_2 + v2_dash*sin(si_2)*dt;

x1 = [x1,x_coordinate_1];
x2 = [x2,x_coordinate_2];

y1 = [y1,y_coordinate_1];
y2 = [y2,y_coordinate_2];

%[AX22,H12,H22] = plotyy(x1,y1,x2,y2,'plot');
%set(get(AX22(1),'Ylabel'),'String','pos_error body 2 ')
%set(get(AX22(2),'Ylabel'),'String','vel_error body 2')

plot(W1(1),W1(2),'x');

hold on
plot(W2(1),W2(2),'x');
plot(x1,y1);
%hold on
plot(x2,y2);
axis([-6 6 -6 6])
axis square

pause(dt);
end

% %AX1 = plot(x2,y2);
% [AX22,H12,H22] = plotyy(x1,y1,x2,y2,'plot');
% set(get(AX22(1),'Ylabel'),'String','pos_error body 2 ')
% set(get(AX22(2),'Ylabel'),'String','vel_error body 2')
% title('Step Response of Error in virtual time using LQR Control')