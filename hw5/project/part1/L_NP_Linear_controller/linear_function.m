function sys_states_dot = f(t,sys_states)
global mc;
mc = 0.5;
mp = 0.1;
global Jp;
Jp=0.006;
global l;
l=0.3; 
delta_theta = 0.05;
delta_x = 0.01;
g = 9.8;

M = (mc+mp)/(mp*l);
L = Jp/(mp*l)+l;
b = 1/(mp*l);
Dx = delta_x/(mp*g*l);
D_theta = delta_theta/(mp*g*l);

alpha = 1/(M*L-1);
A = [0 0 1 0;
     0 0 0 1; 
     0 -g -alpha*L*Dx alpha*D_theta; 
     0 M*g alpha*D_theta -alpha*M*D_theta];

B = [0;0;b*alpha*L;-b*alpha];

p = [-0.002, -0.56+13j, -0.56-13j ,-0.3]; %[-0.5,-1,-1-j,-1+j];
k = place(A,B,p); %scalar

r = [1;0;0;0];
u = -k*(sys_states-r);  %-------------------------
sys_states_dot = A*sys_states + B*u;

end


