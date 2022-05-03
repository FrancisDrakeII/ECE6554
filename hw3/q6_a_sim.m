lambda = 1;
% Reference modle:
Am = [0, 1; -9.0625, -6];
Bm = [0; 1];

% Plant model: xdot = Ax+Bu, y = Cx
omega2 = 12;   %square of omega
delta = 0.2;
A = [0, 1; -omega2, -delta];
B = [0;1];

% Initialization
x_0     = 0;
xdot_0  = 0; 
xm_0    = 0; 
xmdot_0 = 0;
Kx_0    = [0.5 0.3];%(Am - A)/B/lambda; %1
Kr_0    = 0.02;%Bm/B/lambda;       %??????
W_0     = 3; 



states_0 = [x_0, xdot_0, xm_0, xmdot_0, Kx_0, Kr_0, W_0];

% Set time window:
tend = 100;
tspan = [0, tend];		% Time span from 0 to tend seconds.

% Run sim:
[t, states] = ode45( @q6_a_func, tspan, states_0);

% Take out the states:
x1 = states(:,1);
x2 = states(:,2);
xm1 = states(:,3);
xm2 = states(:,4);
kx1 = states(:,5);
kx2 = states(:,6);
kr  = states(:,7);
w   = states(:,8);

e_pos = x1 - xm1;
e_vel = x2 - xm2;

% Reference signal:
%r = 2 * ones(length(t),1); 
r = 2*sin(2*t);

figure(1);
    subplot(3,1,1);
    plot(t, r, t, x1, t, x2, t, xm1, t, xm2);
    xlabel('Time (sec)');
    grid on;
    title('r, X and Xm for r=2');
    legend('r', 'x1', 'x2', 'xm1', 'xm2t');

    subplot(3,1,2);
    plot(t, e_pos, t, e_vel);
    xlabel('Time (sec)');
    grid on;
    title('errors for position and velocity');
    legend('e_position', 'e_velocity');
    
    subplot(3,1,3);
    plot(t, kx1, t, kx2, t, kr, t, w);
    xlabel('Time (sec)');
    grid on;
    title('K_x, K_r, W');
    legend('kx1', 'kx2', 'kr', 'w');
