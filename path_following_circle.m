v = 0.1;
r = 10;

A = [0 1;
     0 0];
B = [0 0;
     v -(v^2)/r];
 
C = [1 0;
     0 1];
D = 0;

states = {'d' 'd_dot'};
inputs = {'u' 'f'};
outputs = {'pos_error'; 'vel_error'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%poles = eig(A)

%  co = ctrb(sys_ss);
%  controllability = rank(co);
 
 %Q = C'*C;
 Q = [1 0;
     0 1];
 %Q(1,1) = 50;

%R = 1; 
 R = [1 0;
     0 1];
 K = lqr(A,B,Q,R);
 
 Ac = (A-B*K);
 Bc = B;
 Cc = C;
 Dc = D;
 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};
% 
 sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
 
 X0 = [5 , 1];
% 
% 
% %% for plotting
 t = 0:0.01:50; 
 u_1 =[0.2*ones(size(t)) ; ones(size(t))] ;
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

 [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
 set(get(AX(1),'Ylabel'),'String','pos_error ')
 set(get(AX(2),'Ylabel'),'String','vel_error ')
 title('Step Response with LQR Control')