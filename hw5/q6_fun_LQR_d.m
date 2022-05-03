function sys_states_dot = f(t, sys_states)

global x_lqr_dot_matrix;
global cnt2;

A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];

R = 2;
[K,S] = lqr(A,B,Q,R,0);

x = sys_states(1:2);   % x = 2*1

u = -K*x;

x_dot = A*x + B*u;

x_lqr_dot_matrix(cnt2,:) = x_dot;
cnt2 = cnt2 + 1;

sys_states_dot = [x_dot; 0; 0; 0; 0];
end
