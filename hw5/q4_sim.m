% Plant
a = 2; b = 3/2;
% Reference
am = -2; bm = 3;
% Estimated plant
a_hat = 1; b_hat = 2;

% Initialization
xm_0 = 0;
x_0 = 0;
kx_0 = (am-a_hat)/b_hat;
kr_0 = bm/b_hat;
xhat_0 = 0;

sys_states_0 = [x_0, xm_0,xhat_0, kx_0, kr_0];

% Time
tmax = 50;
tspan = [0, tmax];

% Simulation
[t, sys_states] = ode45(@q4_ode, tspan, sys_states_0);

% Taking out the sys_states
x = sys_states(:,1);
xm = sys_states(:,2);
xhat = sys_states(:,3);
kx = sys_states(:,4);
kr = sys_states(:,5);


figure(1);
    subplot(3,1,1);
    plot(t, x, t, xm);
    xlabel('Time(sec)');
    ylabel('States');
    title('x and xm with \gamma_x = 1, \gamma_r = 1, r(t) = 4sin(3t)')
    grid on;

    subplot(3,1,2);
    plot(t, kx);
    xlabel('Time(sec)');
    title('Kx')
    grid on;

    subplot(3,1,3);
    plot(t, kr);
    xlabel('Time(sec)');
    title('Kr')
    grid on;


