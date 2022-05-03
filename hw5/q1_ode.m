function system_states_dot = f(t, system_states) 
global Noise_1;
% Plant
a = 2; 
b = 3/2;
% Reference
am = -2; 
bm = 3;
% Adaptive control gains
global gamma_x; global gamma_r;
gamma_x = 0.1; 
gamma_r = 0.1;
% Reference 
global r;
% r = 5;
r = 4*sin(3*t);
% Taking out the states accordingly
x  = system_states(1);
xm = system_states(2);
kx = system_states(3);
kr = system_states(4);
% error
e = x - xm;
%epsilon
epsilon = 1e-4;
%control input
u = kx*x + kr*r;
tmax = 20;
T=0:0.001:tmax;
noise_1 = interp1(T,Noise_1,t,'nearest');

x_dot = a*(x+noise_1)+b*u;
xm_dot = am*xm+bm*r;
if abs(e)>epsilon
    kx_dot = -gamma_x*(x+noise_1)*e;
else 
    kx_dot = 0;
end
if abs(e)>epsilon
    kr_dot = -gamma_r*r*e;
else
    kr_dot = 0;
end

% kx_dot = -gamma_x*x*e*sign(b);
% kr_dot = -gamma_r*r*e*sign(b);
% Putting the states together and return
system_states_dot = ...
    [x_dot; xm_dot; kx_dot; kr_dot];
end