A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];

x0 = [1.5; 2];

[X, L, G] = care(A, B, Q);

P = X;


tspan = [0,20];
[t,states] = ode45(@q6_fun_f, tspan, x0);
x = states;

[t_lqr, lqr_states] = ode45(@q6_fun_LQR_f, tspan, x0);
x_lqr = lqr_states;

epsilon = 1.5;  
disp(['epsilon: ',num2str(epsilon)]);

norm_x = norm(x);
disp(norm_x)

norm_x_lqr = norm(x_lqr);
disp(norm_x_lqr)


