% Time
tmax = 100;
tspan = [0, tmax];

x_0 = 0;
theta_0 = 0.01;
x_dot_0 = 0;
theta_dot_0 = 0;

global Jp;

global l;
sys_states_0 = [x_0, theta_0, x_dot_0, theta_dot_0];


[t, sys_states] = ode45(@linear_P_function, tspan, sys_states_0);

x = sys_states(:,1);
theta = sys_states(:,2);
x_dot = sys_states(:,3);
theta_dot = sys_states(:,4);

figure(1);
    subplot(2,1,1);
    plot(t, theta);
    xlabel('Time(sec)');
    ylabel('Pendulum Theta');
    title('Linear Perturb Linear Controller: theta ')  %---------------------------
    grid on;

    subplot(2,1,2);
    plot(t, x);
    xlabel('Time(sec)');
    ylabel('Cart trajectory');
    title('Linear Perturb Linear Controller: x ')   %---------------------------
    grid on;

saveas(figure(1),'Linear Perturb Linear.jpg');