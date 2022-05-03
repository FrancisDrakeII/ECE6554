
function states_dot = f(t, states)
% Reference modle:
Am = [0, 1; -9.0625, -6];
Bm = [0; 1];

% Plant model: xdot = Ax+Bu, y = Cx
omega2 = 12;   %square of omega
delta = 0.2;
A = [0, 1; -omega2, -delta];
B = [0;1];
C = [1,0];     %y = Cx

% Reference signal:
% r = 0.8;
r = 0.9*sin(t);
beta_hat = -2;

% Adaptive control gains:
Kx_1 = -Am(2,1)-omega2;
Kx_2 = -Am(2,2)-delta;
Kr = -1/(C*inv(A-B*[Kx_1, Kx_2])*B);

% Take out the states:
x = states(1);
x_dot = -beta_hat*x^3 + states(2);
xm = states(3);
xm_dot = states(4);

% Plant input:
u = -[Kx_1, Kx_2]*[x; x_dot] +Kr*r-beta_hat*x^3;

% Update states:
Xdot = A*[x; x_dot] + B*u;
Xmdot = Am*[xm; xm_dot] + Bm*Kr*r;

states_dot = [Xdot; Xmdot];
end