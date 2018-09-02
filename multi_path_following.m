v1 = 0.1;
r1 = 2;
v2 = 0.2;
r2 = 3;

A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];
 
B = [0 0 0 0;
     v1 0 -(v1^2)/r1 0;
     0 0 0 0;
     v2 0 0 -(v2^2)/r2 ];
 
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 
D = 0;

states = {'d1' 'd1_dot' 'd1' 'd1_dot'};
inputs = {'u' 'f' 'f1' 'f2'};
outputs = {'pos_error1'; 'vel_error1';'pos_error1'; 'vel_error1'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%poles = eig(A)

%  co = ctrb(sys_ss);
%  controllability = rank(co);
 
%Q = C'*C;
Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
%Q(1,1) = 50;

 %R = 1;
 R = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];
  
 K  = lqr(A,B,Q,R)
 
 Ac = (A-B*K);
 Bc = B;
 Cc = C;
 Dc = D;
 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};
% 
 sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
 
 X0 = [5 , 1 , 5 , 1];
% 
% 
% %% for plotting
 t = 0:0.01:300; 
 u_1 =[0.1*ones(size(t)); ones(size(t)) ; ones(size(t)) ; ones(size(t))] ;
 [y,t,x]=lsim(sys_cl,u_1,t,X0);
%  
%  i =1;
%  for ang=0:(2*pi)/5000:2*pi
%    
%  xp1=(r+y(i,2))*cos(ang);
%  yp1=(r+y(i,2))*sin(ang);
%  i = i+1;
%  plot(xp1,yp1);
%  end
%  
% xp2=r*cos(ang);
% yp2=r*sin(ang);
% plot(x1+xp,y1+yp);
% 
% plot(xp1,yp1);

 figure 
 [AX1,H11,H21] = plotyy(t,y(:,1),t,y(:,2),'plot');
 set(get(AX1(1),'Ylabel'),'String','pos_error ')
 set(get(AX1(2),'Ylabel'),'String','vel_error ')
 title('Step Response with LQR Control')
 
 figure
 [AX2,H12,H22] = plotyy(t,y(:,3),t,y(:,4),'plot');
 set(get(AX2(1),'Ylabel'),'String','pos_error body 2 ')
 set(get(AX2(2),'Ylabel'),'String','vel_error body 2')
 title('Step Response with LQR Control body 2')