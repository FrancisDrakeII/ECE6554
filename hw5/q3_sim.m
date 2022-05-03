close all; clear all;

K = 0.567;
Kd = 0.625;

% Reference
am = [0, 1; -2*K, -2*Kd+0.5]; bm = [0;1];
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
tmax = 20;
tspan = [0, tmax];
STD = 0.25;
global Noise_1; 
T=0:0.001:tmax;
Noise_1 = STD*randn(1,length(T));

% Simulation
[t, sys_states] = ode45(@q3_func, tspan, sys_states_0);

% Taking out the sys_states
x = sys_states(:,1:2);
xm = sys_states(:,3:4);
kx = sys_states(:,5:6);
kr = sys_states(:,7);


figure(1);
    subplot(4,1,1);
    plot(t, x(:,1), t, xm(:,1));
    xlabel('Time(sec)');
    ylabel('States');
    title('x1 and xm1 with gamma_x = .1, gamma_r = .1, r(t) = 1')
    grid on;
    legend ('x1', 'xm1');

    subplot(4,1,2);
    plot(t, x(:,2), t, xm(:,2));
    xlabel('Time(sec)');
    ylabel('States');
    title('x2 and xm2 with gamma_x = .1, gamma_r = .1, r(t) = 1')
    grid on;
    legend ('x2', 'xm2');

    subplot(4,1,3);
    plot(t, kx);
    xlabel('Time(sec)');
    title('Kx')
    grid on;
    legend ('kx1', 'kx2');

    subplot(4,1,4);
    plot(t, kr);
    xlabel('Time(sec)');
    title('Kr')
    grid on;
    