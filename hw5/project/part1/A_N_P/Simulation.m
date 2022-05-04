%----------------------------Linear System NP Adaptive Control--------------------
close all; clear all;

% Time
tmax = 20;
tspan = [0, tmax];

% Initialization
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

alpha = 1/(M*L-1);

A = [0 0    1                0;
     0 0    0                1;
     0 -g*alpha  -alpha*L*Dx    alpha*Dtheta;
     0 M*g*alpha   alpha*Dx     -alpha*M*Dtheta];

B = [0; 0; b*alpha*L; -b*alpha];

global p;
p=[-2, -0.56+5j, -0.56-5j ,-5];  
k = place(A,B,p);

kx_0 = k; 
kr_0 = -k;

sys_states_0 = [Y_0, Y_P_0, kx_0, kr_0];

% Simulation
[t, sys_states] = ode45(@A_N_P, tspan, sys_states_0);

x = sys_states(:,1);
x_P = sys_states(:,5);
theta = sys_states(:,2);
theta_P = sys_states(:,6);

kx = sys_states(:,9:12);
kr = sys_states(:,13:16);

e = x_P-x;

r = 1;

figure(1);
    subplot(4,1,1);
    plot(t, theta, t, theta_P);
    xlabel('t');
    ylabel('theta');
    title('Adaptive Non-Linear Perturbable Controller: theta')  
    grid on;
    legend('theta', 'thetaP');

    subplot(4,1,2);
    plot(t, x, t, x_P);
    xlabel('t');
    ylabel('Trajectory x');
    title('Adaptive Non-Linear Perturbable Controller: x') 
    grid on;
    legend('x', 'xP')

    subplot(4,1,3);
    plot(t, kx);
    xlabel('t');
    ylabel('kx');
    title('Adaptive Non-Linear Perturbable Controller: kx')   
    grid on;
    legend('1', '2', '3', '4')

    subplot(4,1,4);
    plot(t, kr);
    xlabel('t');
    ylabel('kr');
    title('Adaptive Non-Linear Perturbable Controller: kr')   
    grid on;
    legend('1', '2', '3', '4')

    
saveas(figure(1),'A_N_P.jpg');
