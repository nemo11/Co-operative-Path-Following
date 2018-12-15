syms a q1 q2 q3 q4
fun = @root10d;
x0 = [0,0,0,0,0,0,0,0,0,0];
lb = [0,0,0,0,0,0,0,0,0,0];
options = optimset('MaxFunEvals',1e10);
x = fsolve(fun , x0 , lb)
