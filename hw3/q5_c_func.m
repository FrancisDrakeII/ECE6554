function sys_states_dot = f(t, sys_states) 

sys_states = transpose(sys_states);
%disp(sys_states)

K = 0.567;
Kd = 0.625;

% Reference
%am = [0, 1; -2*K, -2*Kd+0.5]; bm = [0;1];
am = [0, 1; -2*K, -2*Kd]; bm = [0;1];
% Plant
a = [0,1;0, 0.5]; b = [0;2];

% Adaptive control gains
gamma_x = 0.01; gamma_r = 0.01;

% Reference i/p:
% r = 1;         %(c)
% r = sin(t);    %(d)
r = rsteps(t);  %(e)
% r = r(1);      %(e-1)
r = r(2);       %(e-2)

% Taking out the states accordingly

x = transpose(sys_states(1:2));
xm = transpose(sys_states(3:4));
kx = transpose(sys_states(5:6));
kr = sys_states(7);

e = x - xm;
Q = [1, 0; 0, 1];
P = lyap(am,Q);
u = transpose(kx)*x + kr*r;

x_dot = a*x + b*u;
xm_dot = am*xm + bm*kr*r;

kx_dot = -gamma_x*x_dot*transpose(e)*P*b*sign(1); % 2x1 
kr_dot = -gamma_r*r*transpose(e)*P*b*sign(1);      % 2x1

% % Updating the state variables
% % a = 2x2, b = kx = 2x1 , x = 2x1 , kr = 1x1, r= 1x1
% x_dot = (a+b.*kx)*x+ b.*kr.*r;
% xm_dot = am*xm + bm*r;
% kx_dot = transpose(-gamma_x*transpose(b)*P*e*transpose(x)); % .*sign(b);
% kr_dot = -gamma_r*transpose(b) *P*e*r; % .*sign(b);

% Putting the states together and return
sys_states_dot = [(x_dot); (xm_dot); (kx_dot); kr_dot];

end