%----------------------------Noninear System--------------------
%Original nonlinear sys
close all; clear all;

% Time
tmax = 10;
tspan = [0, tmax];

x = 0;
theta = 0.1;
x_dot = 0;
theta_dot = 0;

sys_states_0 = [x, theta, x_dot, theta_dot];

% Simulation
[t, sys_states] = ode45(@L_N_NP, tspan, sys_states_0);

x = sys_states(:,1);
theta = sys_states(:,2);

figure(1);
    subplot(2,1,1);
    plot(t, theta);
    xlabel('Time(sec)');
    ylabel('Pendulum Theta');
    title('Linear Non-Perturbable Non Linear: theta')  %---------------------------
    grid on;

    subplot(2,1,2);
    plot(t, x);
    xlabel('Time(sec)');
    ylabel('Cart trajectory');
    title('Linear Non-Perturbable Non Linear: x')   %---------------------------
    grid on;
saveas(figure(1),'L_N_NP.jpg');