clear all;
close all;
total_time = 10;

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
             0 0 2 0;
             0 0 0 1];

R1_circle = [1 0;
             0 1];
  
K1_circle  = lqr(A1_circle,B1_circle,Q1_circle,R1_circle);

%% agent 3
A2_circle = [0 1 0 0 0;
      0 0 0 0 0;
      0 0 0 0 0;
      0 0 0 0 0;
      0 0 0 0 -1];
 
B2_circle =  [0,0;
              v2_circle_dash,0;
              0,1/radius2;
              0,1/radius2;
              0,0];
 
% u(1) = si_1_dot
% u(2) = v1_dot

Q2_circle = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 2 0 0;
             0 0 0 2 0;
             0 0 0 0 1];

R2_circle = [1 0;
             0 1];
  
K2_circle  = lqr(A2_circle,B2_circle,Q2_circle,R2_circle);
%% agent 2
A3_circle = [0 1 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 -1];
 
B3_circle = [0,0;
             v3_circle_dash,-v3_circle_dash/radius3;
             0,1/radius3;
             0,0];
% u(1) = si_1_dot
% u(2) = v1_dot

Q3_circle = [1 0 0 0;
      0 1 0 0;
      0 0 2 0;
      0 0 0 1];

R3_circle = [1 0;
             0 1];
  
K3_circle  = lqr(A3_circle,B3_circle,Q3_circle,R3_circle);
%%

dt = 0.01;
T = 0:dt:200;

W1_circle = [-radius1 - 20,40,0,0,0];           % x,y,si,centre_x_centre_y
W2_circle = [0,radius2+20,0,0,0];           % x,y,si,centre_x,centre_y
W3_circle = [radius3/sqrt(2),radius3/sqrt(2) + 20,-pi/2,0,0];
x1 = W1_circle(1);
y1 = W1_circle(2);
x2 = W2_circle(1);
y2 = W2_circle(2);
x3 = W3_circle(1);
y3 = W3_circle(2);

figure
title('Step Response of Error in virtual time using LQR Control')

qwerty = size(T);

si_1 = W1_circle(3);
x_coordinate_1 = W1_circle(1);
y_coordinate_1 = W1_circle(2);

si_2 = W2_circle(3);
if si_2 > 2*1.57
    si_2 = -(2*pi - si_2);
end
if si_2 < -2*1.57
    si_2 = (2*pi + si_2);
end
x_coordinate_2 = W2_circle(1);
y_coordinate_2 = W2_circle(2);

si_3 = W3_circle(3);
x_coordinate_3 = W3_circle(1);
y_coordinate_3 = W3_circle(2);

v1 = [0;0];     %angular veocity ; linear velocity
v2 = [0;0];
v3 = [0;0];

dt = 0.01;
T = 0:dt:total_time;


for c = 1:2000
%     t = T(c); 

    R1_circle = sqrt((x_coordinate_1 - W1_circle(4))^2 + (y_coordinate_1 - W1_circle(5))^2);
    R2_circle = sqrt((x_coordinate_2 - W2_circle(4))^2 + (y_coordinate_2 - W2_circle(5))^2);
    R3_circle = sqrt((x_coordinate_3 - W3_circle(4))^2 + (y_coordinate_3 - W3_circle(5))^2);
    
    theta_1 = atan2(y_coordinate_1,x_coordinate_1);
    theta_2 = atan2(y_coordinate_2,x_coordinate_2);
    theta_3 = atan2(y_coordinate_3,x_coordinate_3);

    alpha1 = theta_1 - 1.57;
%     if theta_1 < -pi/2 && theta_1 >= -pi
%         alpha1 = 3*pi/2 + theta_1;
%     end
    alpha2 = -(pi/2 - theta_2);
%     if theta_2 < -pi/2 && theta_2 >= -pi
%         alpha2 = 3*pi/2 + theta_2;
%     end
    alpha3 = theta_3 - 1.57;
%     if theta_3 < -pi/2 && theta_3 >= -pi
%         alpha3 = 3*pi/2 + theta_3;
%     end

    %% Agent1
    cor1 = theta_1 -theta_2;
    z1 = 1 - v1(2)/v1_circle_dash ;
    if cor1 > 2*1.57
        cor1 = -(2*pi - cor1);
    end
    if cor1 < -2*1.57
        cor1 = (2*pi + cor1);
    end
    d1 = R1_circle-radius1;
    Vd1 = v1(2)*sin(si_1-alpha1);
    
    %% Agent2
    cor2_1 = theta_2 -theta_1;
        if cor2_1 > 2*1.57
            cor2_1 = -(2*pi - cor2_1);
        end
        if cor2_1 < -2*1.57
            cor2_1 = (2*pi + cor2_1);
        end    
    cor2_2 = theta_2 -theta_3;
        if cor2_2 > 2*1.57
            cor2_2 = -(2*pi - cor2_2);
        end
        if cor2_2 < -2*1.57
            cor2_2 = (2*pi + cor2_2);
        end    
    z2 =1 - v2(2)/v2_circle_dash;
    d2 = R2_circle-radius2;
    Vd2 = v2(2)*sin(si_2-alpha2);
    cor2 = [ cor2_1 ; cor2_2];
    %% Agent3
    cor3 = theta_3 -theta_2;
        if cor3 > 2*1.57
            cor3 = -(2*pi - cor3);
        end
        if cor3 < -2*1.57
            cor3 = (2*pi + cor3);
        end
    z3 =1 - v3(2)/v3_circle_dash;
    d3 = R3_circle-radius3;
    Vd3 = v3(2)*sin(si_3-alpha3);

    %% States and control Variable
    
    X1 = [d1 ; Vd1 ; rad2deg(cor1) ; z1]
    X2 = [d2 ; Vd2 ; rad2deg(cor2_1) ; rad2deg(cor2_2) ; z2];
    X3 = [d3 ; Vd3 ; rad2deg(cor3) ; z3];
            
    %v = u - (v1_dash)/radius1;
    u1 = -K1_circle*X1;
    v1 = u1 - [v1_circle_dash/radius1;v1_circle_dash];

    u2 = -K2_circle*X2;
    v2 = u2 - [v2_circle_dash/radius2;v2_circle_dash];
    
    u3 = -K3_circle*X3;
    v3 = u3 - [v3_circle_dash/radius3;v3_circle_dash];
    
    if v2(2) < 0
        v2(2)= -v2(2);
    end
    if v1(2) < 0
        v1(2)= -v1(2);
    end
    if v3(2) < 0
        v3(2)= -v3(2);
    end
%     if v2(2) < 15
%         v2(2)= 15;
%     end
    
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

    plot(W1_circle(1),W1_circle(2),'x');
    hold on
    xlabel('X(m)');
    ylabel('Y(m)');
    plot(0,0,'o');
    plot(W2_circle(1),W2_circle(2),'x');
    plot(W3_circle(1),W3_circle(2),'x');
    plot(x1,y1,'LineWidth',2);
    %hold on
    plot(x2,y2,'LineWidth',2);
    plot(x3,y3,'LineWidth',2);
    axis([-110 110 -110 110])
    %legend({'Agent 1','Agent 2','Agent 3'},'Location','southwest')
    axis square

    pause(dt);
end
