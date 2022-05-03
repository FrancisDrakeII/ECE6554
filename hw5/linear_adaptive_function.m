function sys_states_dot = f(t, sys_states) 

%-------------------- Non Perturb System 
%reference model
mc = 0.5; %kg
mp = 0.1; %kg
Jp = 0.006; %kg m^2
l = 0.3; %m
delta_theta = 0.05; %N m/rad s
delta_x = 0.01; %N m/s
g = 9.8; %m/s^2

M = (mc+mp)/(mp*l);
L = Jp/(mp*l)+l;
b = 1/(mp*l);
Dx = delta_x/(mp*g*l);
Dtheta = delta_theta/(mp*g*l);

alpha = 1/(M*L-1);

A = [0 0    1                0;
     0 0    0                1;
     0 -g  -alpha*L*Dx    alpha*Dtheta;
     0 M*g   alpha*Dx     -alpha*M*Dtheta];

B = [0; 0; b*alpha*L; -b*alpha];

%-------------------- Perturb System 
% Estimated Plant
mc_P = 0.5; %kg
mp_P = 0.1; %kg
Jp_P = 7.2e-3;
l_p = 0.24;
delta_theta_P = 0.05; %N m/rad s
delta_x_P = 0.01; %N m/s
g_P = 9.8; %m/s^2

M_P = (mc+mp)/(mp*l);
L_P = Jp/(mp*l)+l;
b_P = 1/(mp*l);
Dx_P = delta_x/(mp*g*l);
Dtheta_P = delta_theta/(mp*g*l);

alpha_P = 1/(M*L-1);

A_P = [0 0    1                0;
       0 0    0                1;
       0 -g_P  -alpha_P*L_P*Dx_P    alpha_P*Dtheta_P;
       0 M_P*g_P   alpha_P*Dx_P     -alpha_P*M_P*Dtheta_P];

B_P = [0; 0; b_P*alpha_P*L_P; -b_P*alpha_P];

% Adaptive control gains
gamma_x = .1; gamma_r = .1;

% Reference i/p:
r = [1;1;1;1];        %(c)

% x = sys_states(:,1);
% theta = sys_states(:,2);
% x_dot = sys_states(:,3);
% theta_dot = sys_states(:,4);
sys_states = transpose(sys_states);


Y = sys_states(:,1:4);
Y_P = sys_states(:,5:8);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

e = Y_P - Y;
u = kx.*Y_P+kr.*transpose(r);  

% Updating the state variables
Y_P_dot = transpose(A_P*transpose(Y_P) + B_P.*transpose(u));     %4*1 = 4*4 (4*1) + (4*1)*(4*1)
Y_dot = transpose(A*transpose(Y) + B.*r);
kx_dot = -gamma_x*Y_P_dot.*e.*transpose(sign(B_P));
kr_dot = -gamma_r*transpose(r).*e.*transpose(sign(B_P));

% Putting the states together and return
sys_states_dot = [transpose(Y_dot); transpose(Y_P_dot); transpose(kx_dot); transpose(kr_dot)];

end


