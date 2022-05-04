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
     0 -g*alpha  -alpha*L*Dx    alpha*Dtheta;
     0 M*g*alpha   alpha*Dx     -alpha*M*Dtheta];

B = [0; 0; b*alpha*L; -b*alpha];

%-------------------- Perturb System 
% Estimated Plant
mc_P = 0.5; %kg
mp_P = 0.1; %kg
Jp_P = 8e-3;
l_P = 0.2;
delta_theta_P = 0.05; %N m/rad s
delta_x_P = 0.01; %N m/s
g_P = 9.8; %m/s^2

M_P = (mc_P+mp_P)/(mp_P*l_P);
L_P = Jp_P/(mp_P*l_P)+l_P;
b_P = 1/(mp_P*l_P);
Dx_P = delta_x_P/(mp_P*g_P*l_P);
Dtheta_P = delta_theta_P/(mp_P*g_P*l_P);

alpha_P = 1/(M_P*L_P-1);

A_P = [0 0    1                0;
       0 0    0                1;
       0 -g_P*alpha_P  -alpha_P*L_P*Dx_P    alpha_P*Dtheta_P;
       0 M_P*g_P*alpha_P   alpha_P*Dx_P     -alpha_P*M_P*Dtheta_P];

B_P = [0; 0; b_P*alpha_P*L_P; -b_P*alpha_P];

% Adaptive control gains
gamma_x = 10; gamma_r = 10;

% Reference i/p:
r = [sin(t);0;0;0];

sys_states = transpose(sys_states);

Y = sys_states(:,1:4);
Y_P = sys_states(:,5:8);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

e = Y_P - Y;

global p;
k = place(A,B,p);
A = A-B*k;
P = lyap(A,eye(4));
A_P = A_P-B_P*k;

u = kx.*Y_P+ kr.*(transpose(r));  % (1x4) (1*4)

% Updating the state variables
Y_P_dot = transpose(A_P*transpose(Y_P) + B_P.*transpose(u));     %4*1 = 4*4 (1*4) + (4*1)*(4*1)
Y_dot = transpose(A*transpose(Y) + B.*transpose(k).*r);
%Y_dot = transpose(A*transpose(Y) + B.*transpose(k).*r);
kx_dot = -gamma_x*Y_P.*e*P.*transpose(B_P); %1x4 = 1 x 1x4 1x4 1  
kr_dot = -gamma_r*transpose(r).*e*P.*transpose(B_P);

% Putting the states together and return
sys_states_dot = [transpose(Y_dot); transpose(Y_P_dot); transpose(kx_dot); transpose(kr_dot)];

end