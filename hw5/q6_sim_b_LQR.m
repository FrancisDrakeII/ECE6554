A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];
x0 = transpose([1.5, 2]);

tspan = [0,20];
[t_lqr,states_lqr] = ode45(@q6_ode_b_LQR, tspan, x0);
x_lqr = states_lqr;

[t_pwmn,states_pwmn] = ode45(@q6_ode_b_PWMN,tspan,x0);
x_pwmn = states_pwmn;

figure(1)
%     subplot(2,1,1)
    plot(t_lqr, x_lqr);
    xlabel('Time(sec)');
    title('LQR');
    grid on;
    
%     subplot(2,1,2)
    plot(t_pwmn,x_pwmn);
    xlabel('Time(sec)');
    title('PWMN with epsilon = 1.5');
    grid on;

    
    
    