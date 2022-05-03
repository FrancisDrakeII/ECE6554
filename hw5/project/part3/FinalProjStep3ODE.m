clear;
clc;

tspan = [0,5];%5,10

State0 = [0;0.1;0;0]; 
Statem0 = [0;0.1;0;0];


global p;

p=[-0.002, -0.58+13j, -0.58-13j ,-0.3];
kx_0 = [-0.0024  ;-51;   -1.1;  29];  %Initialize from pole placement
kr_0 = 5; 
W0 = 1;
Phi0 =1;

SysState0= [State0; Statem0; kx_0; kr_0;W0;Phi0];

[t, SysState] = ode45(@FinalProjStep3F, tspan, SysState0);

%Plot
x = SysState(:,1);
xm = SysState(:,5);
theta = SysState(:,2);
thetam = SysState(:,6);
kx = SysState(:,9:12);
kr = SysState(:,13);
W = SysState(:,14);
Phi = SysState(:,15);
e = theta-thetam;

figure(1);
    subplot(2,1,1);
    
    plot(t, theta, t, thetam);
    title('state \theta , \theta_m')
    xlabel('t');
    ylabel('theta and thetam');
    legend('theta', 'thetam');
    grid on;

    subplot(2,1,2);
    
    plot(t, x, t, xm);
    title('state x,m')
    xlabel('t');
    ylabel('x and xm');
    legend('x','xm');
    grid on;

figure(2)
    
    subplot(2,1,1);
    
    plot(t, kx);
    title('gain Kx')
    xlabel('t');
    ylabel('kx');
    legend('kx1', 'kx2', 'kx3', 'kx4')
    grid on;

    subplot(2,1,2);
    plot(t, kr);
    title('gain Kr')
    xlabel('t');
    ylabel('kr');
    legend('kr')
    grid on;

figure(3)
    subplot(2,1,1);
    plot(t, e);
    title('error')
    xlabel('t');
    ylabel('e');
    grid on;