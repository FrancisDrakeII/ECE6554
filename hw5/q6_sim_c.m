A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];

x0 = [1.5; 2];

[X, L, G] = care(A, B, Q);

Am = A - B*G;
%P = lyap(Am, Q);
P = X;

tspan = [0,20];
[t,states] = ode45(@q6_ode_c, tspan, x0);
x = states;


[t_lqr, lqr_states] = ode45(@q6_fun_c_LQR, tspan, x0);
x_lqr = lqr_states;

V = zeros(size(x, 1),1);

V_lqr = zeros(size(x_lqr, 1),1);

for i = 1:(size(V,1))
    V(i) = x(i,:)*P*transpose(x(i,:));
end

for j = 1:(size(V_lqr,1))
    V_lqr(j) = x_lqr(j,:)*P*transpose(x_lqr(j,:));
end


figure(1);
%     subplot(2,1,1);
    plot(t, V);
    xlabel('Time(sec)');
    grid on;
    title('V(t) epsilon = 1.5');

%     subplot(2,1,2);
%     plot(t_lqr, V_lqr);
%     xlabel('Time(sec)');
%     grid on;
%     title('V LQR(t)');
    