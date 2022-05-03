K = 0.567;
Kd = 0.625;

% Reference
%am = [0, 1; -2*K, -2*Kd+0.5]; bm = [0;1];
am = [0, 1; -2*K, -2*Kd]; bm = [0;1];
% Plant
a = [0,1;0, 0.5]; b = [0;2];
% % Estimated plant
% a_hat = 1; b_hat = 2;

% Initialization
xm_0 = [0, 0];  % 2*1
x_0 = [0, 0];   % 2*1
kx_0 = transpose(b)*(am-a);   % 1*2
kr_0 = transpose(b)*bm;     % 1*1

sys_states_0 = [x_0, xm_0, kx_0, kr_0];

% Time
tmax = 30;
tspan = [0, tmax];

% Simulation
[t, sys_states] = ode45(@q5_c_func, tspan, sys_states_0);

% Taking out the sys_states
x = sys_states(:,1);
xm = sys_states(:,2);
kx = sys_states(:,3);
kr = sys_states(:,4);

figure(1);
    subplot(3,1,1);
    plot(t, x, t, xm);
    xlabel('Time(s)');
    ylabel('States');
    title('x and xm with gamma_x = .01, gamma_r = .01, r(t) = rstep(t)')
    legend('x', 'xm')
    grid on;

    subplot(3,1,2);
    plot(t, kx);
    xlabel('Time(s)');
    title('Kx')
    grid on;

    subplot(3,1,3);
    plot(t, kr);
    xlabel('Time(s)');
    title('Kr')
    grid on;
    