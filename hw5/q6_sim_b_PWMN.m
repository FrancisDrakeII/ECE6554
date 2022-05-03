A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];
x0 = transpose([1.5, 2]);
[X, L, G] = care(A, B, Q);
Am = A - B*G;
tspan = [0,20];
[t,states] = ode45(@q6_ode, tspan, x0);
x = states;

figure(1)
    plot(t, x);
    xlabel('Time(sec)');
    title('state covergence for epsilon = 1');
    grid on;
    
    