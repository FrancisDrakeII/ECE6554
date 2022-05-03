function system_states_dot = f(t, system_states) 
global Noise_1;global Noise_2;global Noise_3;global Noise_4;
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
% Reference i/p:
global r;
r = 5;
% r = 4*sin(3*t);
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
xm_dot = am*xm+bm*r;%+noise_2;

global kx_star
theta_kx = kx-kx_star;
vector_kx = -gamma_x*x*e;
kx_dot = proj(kx,-x*transpose(e));

global kr_star
theta_kr = kr-kr_star;
vector_kr = -gamma_r*r*e;
kr_dot = proj(theta_kr,vector_kr);
% Putting the states together and return
system_states_dot = ...
    [x_dot; xm_dot; kx_dot; kr_dot];
end


function [psi_theta,grad_psi] = psi(theta)
    epsilon_theta = 1e-3;
    theta_max = 1;
    psi_theta = (theta.^2-theta_max.^2)./(epsilon_theta*theta_max.^2);
    grad_psi = 2*theta./(epsilon_theta*theta_max.^2);
end

function proj_val = proj(theta,vector)
    [psi_theta,grad_psi] = psi(theta);
    if psi_theta <0
        proj_val = vector;
    else
        if psi_theta>=0 && grad_psi*vector <=0
            proj_val = vector;
        else
            proj_val = vector*(1-psi_theta);
        end
    end
end