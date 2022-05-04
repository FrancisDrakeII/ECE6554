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

coeff_P = 1/(M_P*L_P-cos(Y_P(2))^2);

A_P = [0    0       1                      0          ; 
       0    0       0                      1          ;
       0   0   -coeff_P*L_P*Dx_P           coeff_P*Dtheta_P*cos(Y_P(2)); 
       0   0   coeff_P*Dtheta_P*cos(Y_P(2))  -coeff_P*M_P*Dtheta_P];

B_P = [    0     ;
           0     ;
      b_P*coeff_P*L_P;
     -b_P*coeff_P*cos(Y_P(2)) ];

C_P = [ 0    0    0              0 ;
        0    0    0              0;
        0    0    0    coeff_P*L_P*sin(Y_P(2));
        0    0    0    -coeff_P*cos(Y_P(2))*sin(Y_P(2))];
D_P = [ 0;
        0;
        -g*cos(Y_P(2))*sin(Y_P(2))*coeff_P;
        M_P*g*sin(Y_P(2))*coeff_P];


gamma_x = 10; gamma_r = 10;


r = [sin(t);0;0;0];



e = Y_P - Y;

global p;
k = place(A,B,p);
A = A-B*k;
P = lyap(A,eye(4));
A_P = A_P-B_P*k;

u = kx.*Y_P + kr.*(transpose(r));

Y_P_dot = A_P*transpose(Y_P) + B_P.*transpose(u) + C_P*transpose(Y_P.^2) + D_P; 

Y_dot = A*transpose(Y) + B.*transpose(k.*transpose(r));
kx_dot = -gamma_x*Y_P.*(e*P*B_P); 
kr_dot = -gamma_r*transpose(r).*(e*P*B_P);

sys_states_dot = [Y_dot; Y_P_dot; transpose(kx_dot); transpose(kr_dot)];

end