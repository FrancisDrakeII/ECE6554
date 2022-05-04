%----------------------------Noninear System--------------------
%Original nonlinear sys

function sys_states_dot = f(t,sys_states)
mc = 0.5; %kg
mp = 0.1; %kg
Jp = 6e-3; %kg m^2
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


A = [0    0       1                0          ; 
     0    0       0                1          ;
     0   -g*alpha   -alpha*L*Dx    alpha*Dtheta   ; 
     0   M*g*alpha   alpha*Dtheta -alpha*M*Dtheta];

B = [    0     ;
         0     ;
      b*alpha*L;
     -b*alpha ];

C = [ 0    0    0              0 ;
      0    0    0              0;
      0    0    0    alpha*sys_states(2)*L;
      0    0    0    -1*sys_states(2)*L];

p=[-2, -0.56+5j, -0.56-5j ,-5];  
k = place(A,B,p);
A = A-B*k;
global noise;

u = -k*(sys_states);
sys_states_dot = A*(sys_states+noise) + B*u + C*sys_states.^2;

end