% Reference modle:
Am = [0, 1; -9.0625, -6];
Bm = [0; 1];

% Plant model: xdot = Ax+Bu, y = Cx
omega2 = 12;   %square of omega
delta = 0.2;
A = [0, 1; -omega2, -delta];
B = [0;1];
C = [1,0];     %y = Cx

% Adaptive control gains:
Kx_1 = -Am(2,1)-omega2;
Kx_2 = -Am(2,2)-delta;

% Initialization
x_0 = 0; xdot_0 = 0; xm_0 = 0; xmdot_0 = 0;
states_0 = [x_0, xdot_0, xm_0, xmdot_0];

% Set time window:
tend = 10;
tspan = [0, tend];		% Time span from 0 to tend seconds.

% Run sim:
[t, states] = ode45( @q4_cancel_nonlinear_func, tspan, states_0);

% Take out the states:
x1 = states(:,1);
x2 = states(:,2);
xm1 = states(:,3);
xm2 = states(:,4);

e_pos = x1 - xm1;
e_vel = x2 - xm2;

% Reference signal:
% r = 0.8* ones(length(t),1); 
r = 0.9*sin(t);

figure(1);
    subplot(2,1,1);
    plot(t, r, t, x1, t, x2, t, xm1, t, xm2);
    xlabel('Time (sec)');
    grid on;
    title('r, X and Xm for r = 0.9*sin(t)');
    legend('r', 'x', 'xdot', 'xm', 'xmdot');

    subplot(2,1,2);
    plot(t, e_pos, t, e_vel);
    xlabel('Time (sec)');
    grid on;
    title('errors for position and velocity');
    legend('e_position', 'e_velocity');
