function sys_states_dot = f(t, sys_states) 

%Non Perturb System 
global noise;
sys_states = transpose(sys_states+noise);

Y = sys_states(:,1:4);
Y_P = sys_states(:,5:8);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

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

coeff = 1/(M*L-1);

A = [0    0       1                0          ; 
     0    0       0                1          ;
     0   -g*coeff   -coeff*L*Dx    coeff*Dtheta   ; 
     0   M*g*coeff   coeff*Dtheta -coeff*M*Dtheta];

B = [    0     ;
         0     ;
      b*coeff*L;
     -b*coeff ];

C = [ 0    0    0              0 ;
      0    0    0              0;
      0    0    0    coeff*Y(2)*L;
      0    0    0    -1*Y(2)*L];

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

coeff_P = 1/(M_P*L_P-1);

A_P = [0    0       1                0          ; 
     0    0       0                1          ;
     0   -g_P*coeff_P   -coeff_P*L_P*Dx_P    coeff_P*Dtheta_P   ; 
     0   M_P*g_P*coeff_P   coeff_P*Dtheta_P -coeff_P*M_P*Dtheta_P];

B_P = [    0     ;
         0     ;
      b_P*coeff_P*L_P;
     -b_P*coeff_P ];

C_P = [ 0    0    0              0 ;
      0    0    0              0;
      0    0    0    coeff_P*Y_P(2)*L_P;
      0    0    0    -Y_P(2)*L_P];


gamma_x = 10; gamma_r = 10;


r = sin(t);



e = Y_P - Y;

global p;
k = place(A,B,p);
A = A-B*k;
P = lyap(A,eye(4));
A_P = A_P-B_P*k;

u = kx.*Y_P + kr*r; 

Y_P_dot = transpose(A_P*transpose(Y_P) + B_P.*transpose(u)) + C_P*transpose(Y_P.^2);     
Y_P_dot = Y_P_dot(1,:);

Y_dot = transpose(A*transpose(Y) + B.*transpose(k)*r)+ C*transpose(Y.^2); 
Y_dot = Y_dot(1,:);

kx_dot = -gamma_x*Y_P.*e*P.*transpose(sign(B_P)); 
kr_dot = -gamma_r*[r r r r].*e*P.*transpose(sign(B_P));

sys_states_dot = [transpose(Y_dot); transpose(Y_P_dot); transpose(kx_dot); transpose(kr_dot)];

end