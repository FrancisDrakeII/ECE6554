function sys_states_dot = f(t, sys_states) 
% Plant
a = 5; b = 3;
% Reference
am = -4; bm = 4;

% Adaptive control gains
gamma_x = 0.1; gamma_r = 0.1;
gamma_u = 0.1;
% Reference i/p:
% r = 5;
r = 4*sin(2*t);

% Taking out the states accordingly
x = sys_states(1);
xm = sys_states(2);
kx = sys_states(3);
kr = sys_states(4);
ku = sys_states(5);

uc = kx*x + kr*r;

rmax = 4;
% abs(am/bm) > abs(a/b)*rmax/u_max; u_max> 20/3
global u_max;
u_max = 9;

if abs(uc)<u_max
    u = uc;
else
    u = u_max*sign(uc);
end

delta_u = u - uc;
e = x - xm;
% Updating the state variables
x_dot = (a+b*kx)*x+b*kr*r+b*delta_u;
xm_dot = am*xm+bm*(r+ku*delta_u);
kx_dot = -gamma_x*x*e*sign(b);
kr_dot = -gamma_r*r*e*sign(b);
ku_dot = gamma_u * delta_u*e*bm;

% Putting the states together and return
sys_states_dot = [x_dot; xm_dot; kx_dot; kr_dot; ku_dot];

end