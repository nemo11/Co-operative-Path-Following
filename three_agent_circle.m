clear all;
close all;
total_time = 10;

radius1 = 65;
radius2 = 80;
radius3 = 100;

v1_dash = (2*pi*radius1)/total_time;
v2_dash = (2*pi*radius2)/total_time;
v3_dash = (2*pi*radius3)/total_time;

%% agent 1
A1 = [0 0 0;
      0 0 1;
      0 0 0];
 
B1 = [0,1/radius1;
      0,0;
      v1_dash,-(v1_dash*2)/radius1];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q1 = [10 0 0;
      0 10 0;
      0 0 1];

R1 = [1 0;
      0 1];
  
K1  = lqr(A1,B1,Q1,R1);

%% agent 3
A3 = [0 0 0;
      0 0 1;
      0 0 0];
 
B3 = [0,1/radius3;
      0,0;
      v3_dash,-(v3_dash*2)/radius3];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q3 = [1 0 0;
      0 10 0;
      0 0 1];

R3 = [1 0;
      0 1];
  
K3  = lqr(A3,B3,Q3,R3);
%% agent 2
A2 = [0 0 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
 
B2 = [0,1/radius2;
      0,1/radius2;
      0,0;
      v2_dash,-(v2_dash*2)/radius2];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q2 = [10 0 0 0;
      0 10 0 0;
      0 0 10 0;
      0 0 0 1];

R2 = [1 0;
      0 1];
  
K2  = lqr(A2,B2,Q2,R2);
%%

dt = 0.01;
T = 0:dt:200;

W1 = [radius1,0,-pi/2,0,0];           % x,y,si,centre_x_centre_y
W2= [0,radius2,0,0,0];           % x,y,si,centre_x,centre_y
W3 = [0,-radius3,pi,0,0];
x1 = W1(1);
y1 = W1(2);
x2 = W2(1);
y2 = W2(2);
x3 = W3(1);
y3 = W3(2);

figure
title('Step Response of Error in virtual time using LQR Control')

qwerty = size(T);

si_1 = W1(3);
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

si_3 = W3(3);
x_coordinate_3 = W3(1);
y_coordinate_3 = W3(2);

v1 = [0;0];     %angular veocity ; linear velocity
v2 = [0;0];
v3 = [0;0];

dt = 0.01;
T = 0:dt:total_time;


for c = 1:2000
    t = T(c); 

    R1 = sqrt((x_coordinate_1 - W1(4))^2 + (y_coordinate_1 - W1(5))^2);
    R2 = sqrt((x_coordinate_2 - W2(4))^2 + (y_coordinate_2 - W2(5))^2);
    R3 = sqrt((x_coordinate_3 - W3(4))^2 + (y_coordinate_3 - W3(5))^2);
    
    theta_1 =atan2(y_coordinate_1,x_coordinate_1);
    theta_2 = atan2(y_coordinate_2,x_coordinate_2);
    theta_3 =atan2(y_coordinate_3,x_coordinate_3);

    alpha1 = theta_1 - 1.57;
    alpha2 = -(pi/2 - theta_2);
    if theta_2 < -pi/2 && theta_2 >= -pi
        alpha2 = 3*pi/2 + theta_2;
    end
    alpha3 = theta_3 - 1.57;

    %% Agent1
    cor1 = theta_1 -theta_2;
    d1 = R1-radius1;
    Vd1 = v1(2)*sin(si_1-alpha1);
    
    %% Agent2
    cor2_1 = theta_2 -theta_1;
    cor2_2 = theta_2 -theta_3;
    d2 = R2-radius2;
    Vd2 = v2(2)*sin(si_2-alpha2);
    
    %% Agent3
    cor3 = theta_3 -theta_2;
    d3 = R3-radius3;
    Vd3 = v3(2)*sin(si_3-alpha3);

    %% States and control Variable
    
    X1 = [0 ; d1 ; Vd1];
    X2 = [0 ; 0 ; d2 ; Vd2];
    X3 = [0 ; d3 ; Vd3];
            
    u1 = -K1*X1;
    v1 = u1 - [0;(v1_dash)/2];

    u2 = -K2*X2;
    v2 = u2 - [0;(v2_dash)/2];
    
    u3 = -K3*X3;
    v3 = u3 - [0;(v3_dash)/2];
    
    if v2(2) < 0
        v2(2)= -v2(2);
    end
    if v1(2) < 0
        v1(2)= -v1(2);
    end
    if v3(2) < 0
        v3(2)= -v3(2);
    end
    
    %% updating states
    si_2 = si_2 + v2(1)*dt;
    if si_2 > 2*1.57
        si_2 = -(2*pi - si_2);
    end
    if si_2 < -2*1.57
        si_2 = (2*pi + si_2);
    end
    x_coordinate_2 = x_coordinate_2 + v2(2)*cos(si_2)*dt;
    y_coordinate_2 = y_coordinate_2 + v2(2)*sin(si_2)*dt;
    
    
    si_1 = si_1 + u1(1)*dt;
    if si_1 > 2*1.57
        si_1 = -(2*pi - si_1);
    end
    if si_1 < -2*1.57
        si_1 = (2*pi + si_1);
    end
    x_coordinate_1 = x_coordinate_1 + v1(2)*cos(si_1)*dt;
    y_coordinate_1 = y_coordinate_1 + v1(2)*sin(si_1)*dt;
    
   
    si_3 = si_3 + u3(1)*dt;
    if si_3 > 2*1.57
        si_3 = -(2*pi - si_3);
    end
    if si_3 < -2*1.57
        si_3 = (2*pi + si_3);
    end
    x_coordinate_3 = x_coordinate_3 + v3(2)*cos(si_3)*dt;
    y_coordinate_3 = y_coordinate_3 + v3(2)*sin(si_3)*dt;

    x1 = [x1,x_coordinate_1];
    x2 = [x2,x_coordinate_2];
    x3 = [x3,x_coordinate_3];

    y1 = [y1,y_coordinate_1];
    y2 = [y2,y_coordinate_2];
    y3 = [y3,y_coordinate_3];

    plot(W1(1),W1(2),'x');
    hold on
    plot(0,0,'o');
    plot(W2(1),W2(2),'x');
    plot(W3(1),W3(2),'x');
    plot(x1,y1);
    %hold on
    plot(x2,y2);
    plot(x3,y3);
    axis([-110 110 -110 110])
    axis square

    pause(dt);
end
