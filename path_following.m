
A = [0 1;
     0 0];
B = [0;
     1];
 
C = [1 0;
     0 1];
D = 0;

states = {'d' 'd_dot'};
inputs = {'u'};
outputs = {'pos_error'; 'vel_error'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%poles = eig(A)

%  co = ctrb(sys_ss);
%  controllability = rank(co);
 
 %Q = C'*C;
 Q = [1 0;
     0 1];
 %Q(1,1) = 50;

 R = 1;
 K = lqr(A,B,Q,R)
 
 Ac = (A-B*K);
 Bc = B;
 Cc = C;
 Dc = D;
 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};
% 
 sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
 
 X0 = [0.5 , 0.5];
% 
% 
% %% for plotting
 t = 0:0.01:10;
 r =0.2*ones(size(t));
 [y,t,x]=lsim(sys_cl,r,t,X0);
 [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
 set(get(AX(1),'Ylabel'),'String','pos_error ')
 set(get(AX(2),'Ylabel'),'String','vel_error ')
 title('Step Response with LQR Control')