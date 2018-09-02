% % t = 0:0.01:10;
% % r =0.2*ones(size(t));
% 
% x1 = 0;
% y1 = 0;
% 
% y2 = 1;
% y2 = 1;
% r = 2;
 ang=0:0.01:2*pi; 
% xp1=r*cos(ang);
% yp1=r*sin(ang);
%  
% xp2=r*cos(ang);
% yp2=r*sin(ang);
% plot(x1+xp,y1+yp);
% 
% plot(x1+xp,y1+yp);



p=[1 -1];
q=[0 0];

figure
hold on
  for i=1:length(p)
      xp=2*cos(ang);
      yp=2*sin(ang);
      xunit=xp + p(i);
      yunit=yp + q(i);
      plot(xunit, yunit)
  end