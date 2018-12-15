%syms a q1 q2 q3 q4
x = sym('x', [1,10]);

 assume(x(4) > 0);
 assumeAlso(x(3) > 0);
 assumeAlso(x(2) > 0);
 assumeAlso(x(1) > 0);
assumeAlso(x(5) > 0);
 assumeAlso(x(6) > 0);
 assumeAlso(x(7) > 0);
assumeAlso(x(8) > 0);
 assumeAlso(x(9) > 0);
 assumeAlso(x(10) > 0);

 
 
a = 2;
q1 = 1;
q2 = 1;
q3 = 1;
q4 = 1;

 
F(1) = x(2)^2 + a*x(3)^2 + a*x(4)^2 - q1;
F(2) = x(2)*x(5) + a*x(3)*x(6) + a*x(4)*x(7) - x(1);
F(3) = x(2)*x(6) + a*x(3)*x(8) + a*x(4)*x(9);
F(4) = x(2)*x(7) + a*x(3)*x(9) + a*x(4)*x(10);

F(5) = x(5)^2 + a*x(6)^2 + a*x(7)^2 - q2 -2*x(2);
F(6) = x(5)*x(6) + a*x(6)*x(8) + a*x(7)*x(9) - x(3);
F(7) = x(5)*x(7) + a*x(6)*x(9) + a*x(7)*x(10) - x(4);

F(8) = x(6)^2 + a*x(8)^2 + a*x(9)^2 - q3;
F(9) = x(6)*x(7) + a*x(8)*x(9) + a*x(9)*x(10);

F(10) = x(7)^2 + a*x(9)^2 + a*x(10)^2 - q4;

Sol = solve(F);
%  Sol_1 = simplify(Sol.x1)
%  Sol_2 = simplify(Sol.x2)
%  Sol_3 = simplify(Sol.x3)
%  
%  Sol_4 = simplify(Sol.x4)
%  Sol_5 = simplify(Sol.x5)
%  Sol_6 = simplify(Sol.x6)
%  
%  Sol_7 = simplify(Sol.x7)
%  Sol_8 = simplify(Sol.x8)
%  Sol_9 = simplify(Sol.x9)
%  
%  Sol_10 = simplify(Sol.x10)