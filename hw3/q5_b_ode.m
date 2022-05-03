
K = 0.567;
Kd = 0.625; 

tspan = [0, 20];
x0 = [0; 0];

[t,x] = ode45( @q5_b_func, tspan, x0);

figure(1); hold on;
y = 2*K*x(:,1) + 2*Kd*x(:,2);

plot(t, y);
title('step response of actual system');
xlabel('Time(s)');
ylabel('Amplitude');
grid on;