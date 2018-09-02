clear all;
close all;
total_time = 9;
radius1 = 2;
radius2 = 4;
v1_dash = (2*pi*radius1)/total_time;
v2_dash = (2*pi*radius2)/total_time;


A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];
 
B = [0 0;
     v1_dash 0;
     0 0;
     0 v2_dash];
  
% C = [1 0;
%      0 1];
%  
% D = 0;

Q = [10 0 0 0;
     0 1 0 0;
     0 0 10 0;
     0 0 0 1];

R = [1 0;
     0 1];
  
K  = lqr(A,B,Q,R)  ;

dt = 0.01;
T = 0:dt:total_time;

W1 = [2,0,-pi/2,0,0];           % x,y,si,centre_x,centre_y
W2 = [0,4,0,0,0];           % x,y,si,centre_x,centre_y

x1 = W1(1);
y1 = W1(2);
x2 = W2(1);
y2 = W2(2);

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

qwerty = size(T);

for c = 1:qwerty(2)
    t = T(c); 

    R1 = sqrt((x_coordinate_1 - W1(4))^2 + (y_coordinate_1 - W1(5))^2);
    R2 = sqrt((x_coordinate_2 - W2(4))^2 + (y_coordinate_2 - W2(5))^2);
    
    % 1st quadrant
    theta_1 = atan2(y_coordinate_1,x_coordinate_1);
    theta_2 = atan2(y_coordinate_2,x_coordinate_2);
    
    alpha1 = -(pi/2 - theta_1);
    alpha2 = -(pi/2 - theta_2);
    
    if theta_1 < -pi/2 && theta_1 >= -pi
        alpha1 = 3*pi/2 + theta_1;
    end
    
    if theta_2 < -pi/2 && theta_2 >= -pi
        alpha2 = 3*pi/2 + theta_2;
    end
    
    %states
    d1 = R1-radius1
    Vd1 = v1_dash*sin(si_1-alpha1);
    
    d2 = R2-radius2;
    Vd2 = v2_dash*sin(si_2-alpha2);
    
    
    X = [d1 ; Vd1 ; d2 ; Vd2];
    u = -K*X;
    v = u - [(v1_dash)/radius1;(v2_dash)/radius2];
    
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
    
    x1 = [x1,x_coordinate_1];
    y1 = [y1,y_coordinate_1];
    
    plot(W1(1),W1(2),'x');
    hold on
    plot(0,0,'o');
    plot(x1,y1);
    axis([-6 6 -6 6])

pause(dt);
end
