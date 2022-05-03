function sys_states_dot = f(t, sys_states)

A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];

[X, L, G] = care(A, B, Q);

Am = A - B*G;
P = X;

epsilon = 1.5;   

%temp = Am*sys_states;
x = sys_states;   % x = 2*1

V = transpose(x)*P*x;   % V = 1*1

phi_0 = transpose(x)*(transpose(A)*P + P*A)*x + epsilon*V;   % 1*1
phi_1 = 2*transpose(x)*P*B;   % 1*1

if phi_0 >0
    u = -(phi_0*phi_1)/(transpose(phi_1)*phi_1);
else
    u = 0;
end

x_dot = A*x + B*u;

sys_states_dot = [x_dot];
end
