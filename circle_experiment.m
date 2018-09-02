clear all;
close all;
total_time = 10;
radius2 = 80;
v2_dash = (2*pi*radius2)/total_time;


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
  
K  = lqr(A2,B2,Q2,R2)
% 
% A = [0 0 0 0;
%      0 0 0 0
%      0 0 0 1;
%      0 0 0 0];
%  
% B2 = [0,1/radius1;
%       0,1/radius1;
%       0,0;
%       v1_dash,-(v1_dash*2)/radius1];
% %   
% % B = [0;
% %      v1_dash];
% %  
% % C = [1 0;
% %      0 1];
% %  
% % D = 0;
% 
% Q = [10 0 0 0;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];
% 
% R = [1 0;
%      0 1];
%   
% K  = lqr(A,B2,Q,R)  ;

dt = 0.01;
T = 0:dt:total_time;

W2= [0,radius2,pi,0,0];           % x,y,si,centre_x,centre_y

x2 = W2(1);
y2 = W2(2);

si_2 = W2(3);
if si_2 > 2*1.57
    si_2 = -(2*pi - si_2);
end
if si_2 < -2*1.57
    si_2 = (2*pi + si_2);
end
x_coordinate_2 = W2(1);
y_coordinate_2 = W2(2);

v = [0;10];

qwerty = size(T);

for c = 1:2000
%     t = T(c); 

    R2 = sqrt((x_coordinate_2 - W2(4))^2 + (y_coordinate_2 - W2(5))^2);
   
    
    % 1st quadrant
    theta_2 = atan2(y_coordinate_2,x_coordinate_2)
    
    alpha = -(pi/2 - theta_2);
    
    if theta_2 < -pi/2 && theta_2 >= -pi
        alpha = 3*pi/2 + theta_2;
    end
    
    %states
    d2 = R2-radius2;
    Vd2 = v(2)*sin(si_2-alpha);
    
    X = [0;0;d2 ; Vd2];
    u = -K*X;
    %v = u - (v1_dash)/radius1;
    v = u - [0;(v2_dash)/2];
    
    if v(2) < 0
        v(2)= -v(2);
    end
    
    %updating states
    si_2 = si_2 + v(1)*dt;
    if si_2 > 2*1.57
        si_2 = -(2*pi - si_2);
    end
    if si_2 < -2*1.57
        si_2 = (2*pi + si_2);
    end
    x_coordinate_2 = x_coordinate_2 + v(2)*cos(si_2)*dt;
    y_coordinate_2 = y_coordinate_2 + v(2)*sin(si_2)*dt;
    
    x2 = [x2,x_coordinate_2];
    y2 = [y2,y_coordinate_2];
    
    plot(W2(1),W2(2),'x');
    hold on
    plot(0,0,'o');
    plot(x2,y2);
    axis([-90 90 -90 90])

pause(dt);
end
