function states_dot = f(t, states)
%disp(states);
lambda = 1;

% Reference modle:
Am = [0, 1; -9.0625, -6];
Bm = [0; 1];

% Plant model: xdot = Ax+Bu, y = Cx
omega2 = 9;   %square of omega
delta = 0.1;
A = [0, 1; -omega2, -delta];
B = [0;1];

% Estimated Plant model: xdot = Ax+Bu, y = Cx
omega2_hat = 12;   %square of omega
delta_hat = 0.2;
A_hat = [0, 1; -omega2_hat, -delta_hat];
B_hat = [0;1];

% Adaptive control gains
gamma_x = 30; 
gamma_r = 30;
gamma_w = 30;

% Reference signal:
%r = 2;
r = 2*sin(2*t);
beta = 2;
beta_hat = 3;

% % Adaptive control gains:
% Kx_1 = -Am(2,1)-omega2;
% Kx_2 = -Am(2,2)-delta;
% Kr = -1/(C*inv(A-B*[Kx_1, Kx_2])*B);


% Take out the states:
x = states(1);                 %x1
x_dot = -beta*x^3 + states(2); %x2
xm = states(3);
xm_dot = states(4);
Kx1 = states(5);
Kx2 = states(6);
Kr = states(7);
W  = states(8);

Kx = [Kx1, Kx2];


% P
Q = eye(2);
P = lyap(Am,Q);

% Update states:
u = Kx*[x; x_dot] + Kr*r - W*x^3;
Xdot = A*[x; x_dot] + B*u;
Xmdot = Am*[xm; xm_dot] + Bm*Kr*r;

e = Xdot - Xmdot;

Kx_dot = -gamma_x*Xdot*transpose(e)*P*B*sign(lambda); % 2x1 
Kr_dot = -gamma_r*r*transpose(e)*P*B*sign(lambda);      % 2x1
W_dot = gamma_w* x^3 * transpose(e) * P * B * sign(lambda); % 1x1

states_dot = [Xdot; Xmdot; Kx_dot; Kr_dot; W_dot];
end