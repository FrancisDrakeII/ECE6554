
function sys_states_dot = f(t, sys_states) 
% Plant
a = 2; b = 3/2;
% Reference
am = -2; bm = 3;

ap = -3;

% Adaptive control gains
gamma_x = 1; gamma_r = 1;
gamma = 1;

% Reference i/p:
% r = 5;
r = 4*sin(3*t);

% Taking out the states accordingly
x = sys_states(1);
xm = sys_states(2);
xhat = sys_states(3);
kx = sys_states(4);
kr = sys_states(5);

e = x - xm;

x_wave = xhat-x;
% Updating the state variables
x_dot = (a+b*kx)*x + b*kr*r;
xm_dot = am*xm + bm*r;
xhat_dot = ap*(xhat-x)+am*x+bm*r;
kx_dot = -gamma_x*x*(e-gamma*x_wave)*sign(b);
kr_dot = -gamma_r*r*(e-gamma*x_wave)*sign(b);

% Putting the states together and return
sys_states_dot = [x_dot; xm_dot; xhat_dot; kx_dot; kr_dot];

end