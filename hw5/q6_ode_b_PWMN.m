function sys_states_dot = f(t, sys_states)

r = 1;
A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];
[X, L, G] = care(A, B, Q);
Am = A - B*G;
P = X;
x = sys_states;
V = transpose(x)*P*x;


epsilon = 1.5;
phi0 = transpose(x)*(transpose(A)*P+P*A)*x+epsilon*V;
phi1 = 2*transpose(x)*P*B;

if phi0>0
    u = -(phi0*phi1)/(transpose(phi1)*phi1);
else
    u = 0;
end

x_dot = A*x+B*u;
sys_states_dot = [x_dot];
end
