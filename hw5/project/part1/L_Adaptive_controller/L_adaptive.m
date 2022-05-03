function sys_states_dot = f(t, sys_states) 
%Non Perturb System 
%reference model
mc = 0.5; 
mp = 0.1; 
Jp = 0.006; 
l = 0.3;
delta_theta = 0.05; 
delta_x = 0.01; 
g = 9.8; 

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

A = [0 0 1 0;
     0 0 0 1; 
     0 -g -alpha*L*Dx alpha*Dtheta; 
     0 M*g alpha*Dtheta -alpha*M*Dtheta];

B = [0;0;b*alpha*L;-b*alpha];


%Perturb System 
% Estimated Plant
mc_P = 0.5;
mp_P = 0.1; 
Jp_P = 7.2e-3;
l_P = 0.24;
delta_theta_P = 0.05;
delta_x_P = 0.01; 
g_P = 9.8; 

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

%A-linear, need pole placement to get the Am

gamma_x = 10; gamma_r = 10;

% Reference:
r = [1;1;1;1];

sys_states = transpose(sys_states);

Y = sys_states(:,1:4);
Y_P = sys_states(:,5:8);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

e = Y_P - Y;

global p;
k =place(A,B,p);
A = A-B*k;
u = kx.*Y_P+ kr.*transpose(r); 

Y_P_dot = transpose(A_P*transpose(Y_P) + B_P.*transpose(u));   
Y_dot = transpose(A*transpose(Y) + B.*transpose(k).*r); %reference model

kx_dot = -gamma_x*Y_P.*e.*transpose(sign(B_P));  
kr_dot = -gamma_r*transpose(r).*e.*transpose(sign(B_P));  

sys_states_dot = [transpose(Y_dot); transpose(Y_P_dot); transpose(kx_dot); transpose(kr_dot)];

end