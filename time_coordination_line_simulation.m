clear all;
close all;
total_time = 9;
total_distance1 = 20;
total_distance2 = 20;
v1_dash = total_distance1/total_time;
v2_dash = total_distance2/total_time;

A = [0 0 0 0 0;
     0 0 1 0 0;
     0 0 0 0 0;
     0 0 0 0 1;
     0 0 0 0 0];
 
B = [0,0,1,-1;
     0,0,0,0;
     v1_dash,0,0,0;
     0,0,0,0;
     0,v2_dash,0,0];
 
C = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 1 0;
     0 0 0 0 1];
 
D = 0;

states = {'gamma' 'd1' 'd1_dot' 'd2' 'd2_dot'};
inputs = {'u1' 'u2' 'u3' 'u4'};
outputs = {'time_coord'; 'pos_error1'; 'vel_error1';'pos_error2'; 'vel_error2'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 1 0;
     0 0 0 0 1];

R = [1 0 0 0 ;
     0 1 0 0 ;
     0 0 1 0 ;
     0 0 0 1 ];
  
K  = lqr(A,B,Q,R)  ;

% Ac = (A-B*K);
% Bc = B;
% Cc = C;
% Dc = D;

dt = 0.01;
T = 0:dt:total_time;

W1 = [0,2,0,4,0];           % x,y,si,v,si_dot
W2 = [0,4,0,2,0];
x1 = W1(1);
y1 = W1(2);
x2 = W2(1);
y2 = W2(2);

figure
title('Step Response of Error in virtual time using LQR Control')

qwerty = size(T);


si_1 = W1(3);
x_coordinate_1 = W1(1);
y_coordinate_1 = W1(2);
si_2 = W2(3);
x_coordinate_2 = W2(1);
y_coordinate_2 = W2(2);

theta_1 = 0;
theta_2 = 0;

u = [W1(5);W2(5);W1(4);W2(4)];

for c = 1:qwerty(2)

t = T(c); 
    
%R1 = sqrt((x_coordinate_1 - W1(1))^2 + (y_coordinate_1 - W1(2))^2);
%R2 = sqrt((x_coordinate_2 - W2(1))^2 + (y_coordinate_2 - W2(2))^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if u(3) > u(2)
 B = [0,0,1,-1;
      0,0,0,0;
      v1_dash,0,0,0;
      0,0,0,0;
      0,v2_dash,0,0];

% K  = lqr(A,B,Q,R);

else 
 B = [0,0,-1,1;
      0,0,0,0;
      v1_dash,0,0,0;
      0,0,0,0;
      0,v2_dash,0,0];
   
end    

K  = lqr(A,B,Q,R);




alpha = abs(u(3) - u(4));
%d1 = R1*sin(theta_1 - si_1);
d1 = abs(y_coordinate_1 - 2);
Vd1 = u(3)*sin(si_1 - theta_1);
%d2 = R2*sin(theta_2 - si_2);
d2 = abs(y_coordinate_2 - 4);
Vd2 = u(4)*sin(si_2 - theta_2);

X = [alpha ; d1 ; Vd1 ; d2 ; Vd2];

%X_dot = Ac*X;
u = -K*X;

% if u(3) > 4
%     u(3) = 4;
% end
if u(3) < -4
        u(3) = -4;
end
% if u(4) > 4
%     u(4) = 4;
% end
if u(4) < -4
        u(4) = -4;
end
%updating states
si_1 = si_1 + u(1)*dt;
x_coordinate_1 = x_coordinate_1 + u(3)*cos(si_1)*dt;
y_coordinate_1 = y_coordinate_1 + u(3)*sin(si_1)*dt;
si_2 = si_2 + u(2)*dt;
x_coordinate_2 = x_coordinate_2 + u(4)*cos(si_2)*dt;
y_coordinate_2 = y_coordinate_2 + u(4)*sin(si_2)*dt;

x1 = [x1,x_coordinate_1];
x2 = [x2,x_coordinate_2];

y1 = [y1,y_coordinate_1];
y2 = [y2,y_coordinate_2];

%[AX22,H12,H22] = plotyy(x1,y1,x2,y2,'plot');
%set(get(AX22(1),'Ylabel'),'String','pos_error body 2 ')
%set(get(AX22(2),'Ylabel'),'String','vel_error body 2')

plot(W1(1),W1(2),'x');

hold on
    xlabel('X(m)');
    ylabel('Y(m)');
plot(W2(1),W2(2),'x');
plot(x1,y1,'LineWidth',2);
%hold on
plot(x2,y2,'LineWidth',2);
axis([-2 30 0 6])

pause(dt);
end

% %AX1 = plot(x2,y2);
% [AX22,H12,H22] = plotyy(x1,y1,x2,y2,'plot');
% set(get(AX22(1),'Ylabel'),'String','pos_error body 2 ')
% set(get(AX22(2),'Ylabel'),'String','vel_error body 2')
% title('Step Response of Error in virtual time using LQR Control')