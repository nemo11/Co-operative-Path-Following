syms X 
sol = sym('sol', [1 4]);
assume(sol(4) > X);
assumeAlso(sol(4) > sol(3));
assumeAlso(sol(3) > 0);
assumeAlso(X > 0);
F = [ (16*sol(1))/7 + (16*sol(2))/7 - 16/7;
- (16*sol(1)*sol(3))/7 - (16*sol(2)*sol(4))/7;
(16*sol(1)*sol(3)^2)/7 + (16*sol(2)*sol(4)^2)/7 + 2 ];

F = subs(F, sol(3), X);

Sol = solve(F);
Sol_1 = simplify(Sol.sol1)
Sol_2 = simplify(Sol.sol2)
Sol_4 = simplify(Sol.sol4)
