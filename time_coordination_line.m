
total_time = 20;
total_distance1 = 10;
total_distance2 = 20;

A = [0 0 0 0 0;
     0 0 1 0 0;
     0 0 0 0 0;
     0 0 0 0 1;
     0 0 0 0 0];
 
B = [0 0 1 -1;
     0 0 0 0;
     total_distance1/total_time 0 0 0;
     0 0 0 0;
     0 total_distance2/total_time 0 0];
 
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
  
K  = lqr(A,B,Q,R)  

Ac = (A-B*K);
Bc = B;
Cc = C;
Dc = D;
 
 
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
 
X0 = [5 ,5 , 1 , 5 , 1];
t = 0:0.01:total_time; 
u_1 =[0.1*ones(size(t)); 0.1*ones(size(t)) ; ones(size(t)) ; ones(size(t))] ;
[y,t,x]=lsim(sys_cl,u_1,t,X0);

 
 figure 
 AX1 = plot(t,x(:,1));
 title('Step Response of Error in virtual time using LQR Control')
 
 figure
 [AX22,H12,H22] = plotyy(t,y(:,2),t,y(:,3),'plot');
 set(get(AX22(1),'Ylabel'),'String','pos_error body 2 ')
 set(get(AX22(2),'Ylabel'),'String','vel_error body 2')
 title('Step Response with LQR Control body 2')
 
 figure 
 [AX11,H11,H21] = plotyy(t,y(:,4),t,y(:,5),'plot');
 set(get(AX11(1),'Ylabel'),'String','pos_error ')
 set(get(AX11(2),'Ylabel'),'String','vel_error ')
 title('Step Response with LQR Control')
 