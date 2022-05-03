function sys_states_dot = f(t, sys_states) 
% plant Model variable 
a = 2; b = 3/2;
% reference Model variable 
am = -2; bm = 3;

% control gains
gamma_x = 0.1; gamma_r = 0.1;

% r(t)
r = 5;
% r = 4*sin(3*t);

% state manipulation
x = sys_states(1);
xm = sys_states(2);
kx = sys_states(3);
kr = sys_states(4);

e = x - xm;

% update state dot
x_dot = (a+b*kx)*x + b*kr*r;
xm_dot = am*xm + bm*r;
kx_dot = -gamma_x*x*e*sign(b);
kr_dot = -gamma_r*r*e*sign(b);

% form in a new matrix
sys_states_dot = [x_dot; xm_dot; kx_dot; kr_dot];

end
