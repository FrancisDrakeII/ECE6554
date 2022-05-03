function system_states_dot = f(t, system_states) 
global Noise_1;
% Plant
a = 2; 
b = 3/2;
% Reference
am = -2;
bm = 3;


global gamma_x; global gamma_r;
gamma_x = 1; 
gamma_r = 1;

% Reference 
global r;
r = 0.25*cos(2*t);

% Taking out the states accordingly
x  = system_states(1);
xm = system_states(2);
kx = system_states(3);
kr = system_states(4);
psi = system_states(5);
% error
e = x - xm;

Gamma_x = 1;  
lambda= 1; 
delta = 0.1;
Q = eye(1);
P=lyap(am,Q);

d = 3*cos(t);
global gamma_psi;
gamma_psi = 0.1;
noise_1 = 3*cos(3*t);

%control input
u = kx*x + kr*r-psi*tanh(transpose(e)*P*b)*sign(lambda);


x_dot = a*(x+noise_1)+b*(u+d);
xm_dot = am*xm+bm*r;
kx_dot = Gamma_x*proj(kx,-x*transpose(e)*P*b*sign(lambda));
kr_dot = gamma_r*proj(kr,-r*transpose(e)*P*b*sign(lambda));
psi_dot = gamma_psi*proj(psi,transpose(e)*P*b*sign(lambda)*tanh((transpose(e)*P*b)*sign(lambda)/delta));


system_states_dot = ...
    [x_dot; xm_dot; kx_dot; kr_dot; psi_dot];
end

function [psi_theta,grad_psi] = psi_func(theta)
    epsilon_theta = 1e-3;
    theta_max = 10;
    psi_theta = (theta.^2-theta_max.^2)./(epsilon_theta*theta_max.^2);
    grad_psi = 2*theta./(epsilon_theta*theta_max.^2);
end

function proj_val = proj(theta,vector)
    [psi_theta,grad_psi] = psi_func(theta);
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