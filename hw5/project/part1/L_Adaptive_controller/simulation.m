
tmax = 20;
tspan = [0, tmax];


Y_0 = [0, 0.01, 0, 0]; 
Y_P_0 = [0, 0.01, 0, 0];

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

A = [0 0    1                0;
     0 0    0                1;
     0 -g*coeff  -coeff*L*Dx    coeff*Dtheta;
     0 M*g*coeff   coeff*Dx     -coeff*M*Dtheta];

B = [0; 0; b*coeff*L; -b*coeff];

AL = [0 0 1 0; 0 0 0 1; 0 -g -coeff*L*Dx coeff*Dtheta; 0 M*g coeff*Dtheta -coeff*M*Dtheta];
BL = [0;0;b*coeff*L;-b*coeff];

global p;
p=[-0.0017, -0.555+13j, -0.555-13j ,-0.3];
k = place(A,B,p);

kx_0 = k;  
kr_0 = -k;

sys_states_0 = [Y_0, Y_P_0, kx_0, kr_0];

[t, sys_states] = ode45(@L_adaptive, tspan, sys_states_0);

x = sys_states(:,1);
x_P = sys_states(:,5);
theta = sys_states(:,2);
theta_P = sys_states(:,6);
kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

figure(1);
    subplot(4,1,1);
    plot(t, theta, t, theta_P);
    xlabel('t');
    ylabel('theta');
    title('Linear+Adaptive: theta')  
    grid on;
    legend('theta', 'thetaP');

    subplot(4,1,2);
    plot(t, x, t, x_P);
    xlabel('t');
    ylabel('Trajectory x');
    title('Linear+Adaptive: x') 
    grid on;
    legend('x', 'xP')

    subplot(4,1,3);
    plot(t, kx);
    xlabel('t');
    ylabel('kx');
    title('Linear+Adaptive: kx')   
    grid on;
    legend('1', '2', '3', '4')

    subplot(4,1,4);
    plot(t, kr);
    xlabel('t');
    ylabel('kr');
    title('Linear+Adaptive: kr')   
    grid on;
    legend('1', '2', '3', '4')