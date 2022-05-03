function sys_states_dot = f(t, sys_states)

A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];
R = 2;
N=0;
[K,S] = lqr(A,B,Q,R,N);

x = sys_states;

u = -K*x;

x_dot = A*x+B*u;
sys_states_dot = [x_dot];
end
