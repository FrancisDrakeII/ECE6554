function xdot = f(t,x)
%Set params as the instructions
mc = 0.45;%modified this param-10%
mp = 0.12;%modified this param,+20%
% mc = 0.5;
% mp = 0.1;
Jp=0.006;
l=0.3; 
dtheta = 0.05;
dx = 0.01;
g = 9.8;

M = (mc+mp)/mp/l;
L = Jp/mp/l+l;
b = 1/mp/l;
Dx = dx/mp/g/l;
Dtheta = dtheta/mp/g/l;

coeff = 1/(M*L-1);

A = [0    0       1                0          ; 
     0    0       0                1          ;
     0   -g   -coeff*L*Dx    coeff*Dtheta   ; 
     0   M*g   coeff*Dtheta -coeff*M*Dtheta];

B = [    0     ;
         0     ;
      b*coeff*L;
     -b*coeff ];

C = [ 0    0    0              0 ;
      0    0    0              0;
      0    0    0    coeff*x(2)*L;
      0    0    0    -1*x(2)*L];

p = [-0.002, -0.58+13j, -0.58-13j ,-0.3]; %Poles
k = place(A,B,p);

r = [1;0;0;0];%ref
u = -k*(x-r); 
xdot = A*x + B*u + C*x.^2;%C is used to add a second order term
end