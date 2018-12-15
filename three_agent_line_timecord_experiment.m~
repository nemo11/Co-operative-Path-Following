clear all;
close all;
total_time = 10;

radius1 = 65;
radius2 = 80;
radius3 = 100;

v1_dash = 20;
v2_dash = 20;
v3_dash = 20;

%% agent 1 line
A1 = [0 0 0;
    
      0 0 1;
      0 0 0];
 
B1 = [0,1;
      0,0;
      v1_dash,0];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q1 = [1000 0 0;
      0 1 0;
      0 0 1];

R1 = [1 0;
      0 1];
  
K1  = lqr(A1,B1,Q1,R1);

%% agent 3
A3 = [0 0 0;
      0 0 1;
      0 0 0];
 
B3 = [0,1;
      0,0;
      v3_dash,0];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q3 = [1000 0 0;
      0 1 0;
      0 0 1];

R3 = [1 0;
      0 1];
  
K3  = lqr(A3,B3,Q3,R3);
%% agent 2
A2 = [0 0 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
 
B2 = [0,1;
      0,1;
      0,0;
      v2_dash,0];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q2 = [100 0 0 0;
      0 100 0 0;
      0 0 1 0;
      0 0 0 1];

R2 = [1 0;
      0 1];
  
K2  = lqr(A2,B2,Q2,R2);
%%

dt = 0.01;
T = 0:dt:200;

W1 = [radius1/sqrt(2) ,-radius1/sqrt(2),0,0,0];           % x,y,si,centre_x_centre_y
W2 = [radius2/sqrt(2) ,-radius2/sqrt(2),0,0,0];           % x,y,si,centre_x,centre_y
W3 = [radius3/sqrt(2) ,-radius3/sqrt(2),0,0,0];
x1 = W1(1) ;
y1 = W1(2) ;
x2 = W2(1) ;
y2 = W2(2) ;
x3 = W3(1);
y3 = W3(2) ;

figure
title('Step Response of Error in virtual time using LQR Control')

qwerty = size(T);

si_1 = W1(3);
x_coordinate_1 = W1(1);
y_coordinate_1 = W1(2) ;

si_2 = W2(3);
if si_2 > 2*1.57
    si_2 = -(2*pi - si_2);
end
if si_2 < -2*1.57
    si_2 = (2*pi + si_2);
end
x_coordinate_2 = W2(1);
y_coordinate_2 = W2(2) ;

si_3 = W3(3);
x_coordinate_3 = W3(1);
y_coordinate_3 = W3(2) ;

v1 = [0;0];     %angular veocity ; linear velocity
v2 = [0;0];
v3 = [0;0];



for c = 1:2000
%     t = T(c); 

%     R1 = sqrt((x_coordinate_1 - W1(4))^2 + (y_coordinate_1 - W1(5))^2);
%     R2 = sqrt((x_coordinate_2 - W2(4))^2 + (y_coordinate_2 - W2(5))^2);
%     R3 = sqrt((x_coordinate_3 - W3(4))^2 + (y_coordinate_3 - W3(5))^2);
%     
%     theta_1 =atan2(y_coordinate_1,x_coordinate_1);
%     theta_2 = atan2(y_coordinate_2,x_coordinate_2);
%     theta_3 =atan2(y_coordinate_3,x_coordinate_3);

    %% Agent1
    %cor1 = theta_2 -theta_1;
    cor1 = x_coordinate_1 - x_coordinate_2;
    d1 = y_coordinate_1-W1(2);
    Vd1 = v1(2)*sin(si_1);
    
    %% Agent2
    %cor2_1 = theta_1 -theta_2;
    %cor2_2 = theta_2 -theta_3;
    cor2_1 = x_coordinate_2 - x_coordinate_1;
    cor2_2 = x_coordinate_2 - x_coordinate_3;
    d2 = y_coordinate_2 - W2(2);
    Vd2 = v2(2)*sin(si_2);
    
    %% Agent3
    %cor3 = theta_3 -theta_2;
    cor3 = x_coordinate_3 - x_coordinate_2;
    d3 = y_coordinate_3 - W3(2);
    Vd3 = v3(2)*sin(si_3);

    %% States and control Variable
    
    X1 = [cor1 ; d1 ; Vd1];
    X2 = [cor2_1 ; cor2_2 ; d2 ; Vd2];
    X3 = [cor3 ; d3 ; Vd3];
            
    u1 = -K1*X1;
    v1 = u1;

    u2 = -K2*X2;
    v2 = u2;
    
    u3 = -K3*X3;
    v3 = u3;
    
%     if v2(2) < 0
%         v2(2)= -v2(2);
%     end
%     if v1(2) < 0
%         v1(2)= -v1(2);
%     end
%     if v3(2) < 0
%         v3(2)= -v3(2);
%     end
    
%     
    if v2(2) > -20
        v2(2) = -20;
    end
%     if v1(2) < 15
%         v1(2)= 15;
%     end
%     if v3(2) < 20
%         v3(2)= 20;
%     end
    
    
%     if v2(2) > 30 
%         v2(2)= 30;
%     end
%     if v1(2) < 30
%         v1(2)= 30;
%     end
%     if v3(2) < 30
%         v3(2)= 30;
%     end
%     
    %% updating states
    si_2 = si_2 + u2(1)*dt;
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
