function sys_states_dot = f(t,sys_states)
global mc;
mc = 0.5;
mp = 0.1;
global Jp;
Jp=7.2e-3;
global l;
l=0.24; 
delta_theta = 0.05;
delta_x = 0.01;
g = 9.8;

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

p = [-0.0017, -0.555+13j, -0.555-13j ,-0.3]; %[-0.5,-1,-1-j,-1+j];
k = place(A,B,p);

u = -k*(sys_states);  
sys_states_dot = A*sys_states + B*u;

end