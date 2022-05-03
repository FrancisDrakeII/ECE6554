clear;
clc;
load('Noise_tspan_0_10_State0_0.1.mat');
%use 20*20*20 neurons
thetavec = -pi/2:pi/19:pi/2;
xdotvec = -3:6/19:3;
thetadotvec = -2*pi:4*pi/19:2*pi;
[CmatrixX,CmatrixY,CmatrixZ] = meshgrid(thetavec,xdotvec,thetadotvec);
sigmavec =[19/pi;19/6 ;19/pi/4 ];
global RBF;
RBF= RBFclass;
RBF.CmatrixX=CmatrixX;
RBF.CmatrixY=CmatrixY;
RBF.CmatrixZ=CmatrixZ;
RBF.sigmavec=sigmavec;
global Noise;
Noise = randn(4,1);%miu=0,var=1 noise
global EofT;%The training error
EofT = [];
tspan = [0,5];%5,10
global alpha1;
global alpha2;
global alpha3;
State0 = [0;0.6;0;0]; %0.1'.2';.3 .....
Statem0 = [0;0.01;0;0];
alpha1 = alpha_end(1,:);
alpha2 = alpha_end(2,:);
alpha3 = alpha_end(3,:);


global p;

p=[-0.002, -0.58+13j, -0.58-13j ,-0.3];
kx_0 = [-0.0024  ;-51;   -1.1;  29];  %Initialize from pole placement
kr_0 = [0.0024 ;  51;   -29  ; -12]; 

SysState0= [State0; Statem0; kx_0; kr_0;];

[t, SysState] = ode45(@FinalProjStep2F, tspan, SysState0);


% alpha1_end = SysState(end,17:36);
% alpha2_end = SysState(end,37:56);
% alpha3_end = SysState(end,57:76);
% alpha_end = [alpha1_end;alpha2_end;alpha3_end];
% save('Noise_tspan_0_10_State0_0.1.mat','alpha_end')
%Plot
x = SysState(:,1);
xm = SysState(:,5);
theta = SysState(:,2);
thetam = SysState(:,6);
kx = SysState(:,9:12);
kr = SysState(:,13:16);
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
    legend('kr1', 'kr2', 'kr3', 'kr4')
    grid on;

figure(3)
    subplot(2,1,1);
    plot(t, e);
    title('error')
    xlabel('t');
    ylabel('e');
    grid on;
