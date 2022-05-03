% Plant
a = 2; 
b = 2;
% Reference
am = -2; 
bm = 3;
% % Estimated plant
% a_hat = 1; b_hat = 2;

% Initialization
xm_0 = 0; 
x_0 =0;   
a_hat_0 = 0.1;  
b_hat_0 = 0.1;     

sys_states_0 = [x_0, xm_0, a_hat_0, b_hat_0];

% Time
tmax = 20;
tspan = [0, tmax];

% Simulation
[t, sys_states] = ode45(@q5_ode, tspan, sys_states_0);

% Taking out the sys_states
x = sys_states(:,1);
xm = sys_states(:,2);
a_hat = sys_states(:,3);
b_hat = sys_states(:,4);
e = x-xm;

figure(1);
    subplot(4,1,1);
    plot(t, x, t, xm);
    xlabel('Time(sec)');
    ylabel('States');
    title('x and xm with \gamma_x = 20, \gamma_r = 20, r(t) = 4sin(3t)')
    legend('x','xm');
    grid on;

    subplot(4,1,2);
    plot(t, a_hat);
    xlabel('Time(sec)');
    title('a hat')
    grid on;

    subplot(4,1,3);
    plot(t, b_hat);
    xlabel('Time(sec)');
    title('b hat')
    grid on;

    subplot(4,1,4);
    plot(t,e);
    xlabel('Time(sec)');
    title('error')
    grid on;

    