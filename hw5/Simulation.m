%----------------------------Linear System NP Adaptive Control--------------------
close all; clear all;

% Time
tmax = 10;
tspan = [0, tmax];

% Initialization
Y_0 = [0.8, 0.15, 0, 0]; 
Y_P_0 = [0.8, 0.15, 0, 0];   
kx_0 = [-0.0031  -54.4086   -0.0133   -0.8491];  %from pole placement
kr_0 = -[-0.0031  -54.4086   -0.0133   -0.8491];

sys_states_0 = [Y_0, Y_P_0, kx_0, kr_0];


% Simulation
[t, sys_states] = ode45(@linear_adaptive_function, tspan, sys_states_0);


x = sys_states(:,1);
theta = sys_states(:,2);
%x_dot = sys_states(:,3);
%theta_dot = sys_states(:,4);
x_P = sys_states(:,5);
theta_P = sys_states(:,6);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

figure(1);
    subplot(4,1,1);
    plot(t, theta,t,theta_P);
    xlabel('Time(sec)');
    ylabel('Pendulum Theta');
    title('Linear system NP Adaptive: theta')  %---------------------------
    grid on;

    subplot(4,1,2);
    plot(t, x,t,x_P);
    xlabel('Time(sec)');
    ylabel('Cart trajectory');
    title('Linear system NP Adaptive: x')   %---------------------------
    grid on;

    subplot(4,1,3);
    plot(t, kx);
    xlabel('Time(sec)');
    ylabel('Cart trajectory');
    title('Kx')   %---------------------------
    grid on;
    
    subplot(4,1,4);
    plot(t, kr);
    xlabel('Time(sec)');
    ylabel('Cart trajectory');
    title('Kr')   %---------------------------
    grid on;
