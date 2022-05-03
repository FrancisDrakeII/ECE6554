
function sys_states_dot = f(t, sys_states) 
% Plant
a = -1/2; b = 1/2;
% Reference
am = -2; bm = 3;

% Adaptive control gains
gamma_x = 1; gamma_r = 1;
gamma_u = 1;
% Reference i/p:
% r = 5;
r = 4*sin(3*t);

% Taking out the states accordingly
x = sys_states(1);
xm = sys_states(2);
kx = sys_states(3);
kr = sys_states(4);
ku = sys_states(5);

uc = kx*x + kr*r;

u_max = 6;

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