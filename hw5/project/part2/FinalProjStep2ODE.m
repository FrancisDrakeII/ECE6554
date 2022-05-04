clear;
clc;
%use 20*20*20 neurons
thetavec = -pi:2*pi/19:pi;
xdotvec = -2:4/19:2;
thetadotvec = -2*pi:4*pi/19:2*pi;
[CmatrixX,CmatrixY,CmatrixZ] = meshgrid(thetavec,xdotvec,thetadotvec);
sigmavec =[19/pi/2;19/4 ;19/pi/4 ];
global RBF ;
RBF= RBFclass;
RBF.CmatrixX=CmatrixX;
RBF.CmatrixY=CmatrixY;
RBF.CmatrixZ=CmatrixZ;
RBF.sigmavec=sigmavec;
% global Noise;
% Noise = 0.25*randn(4,1);
% load('alpha10.mat');
% load('alpha20.mat');
% load('alpha30.mat');
% global alpha1;
% global alpha2;
% global alpha3;
% alpha1 = alpha10;
% alpha2 = alpha20;
% alpha3 = alpha30;
global EofT;%The training error
EofT = [];
tspan = [0,5];%5,10,15

State0 = [0;0.1;0;0]; %0.1'.2';.3 .....
Statem0 = [0;0.01;0;0];

alpha10=rand(20,1);
alpha20=rand(20,1);
alpha30=rand(20,1);

global p;

p=[-0.002, -0.56+10j, -0.56-10j ,-0.3];
kx_0 = [-0.0017  ;-33;   -0.83;  -0.6992];  %Initialize from pole placement
kr_0 = -[-0.0017  ;-33;   -0.83;  -0.6992]; 

SysState0= [State0; Statem0; kx_0; kr_0;alpha10;alpha20;alpha30];

[t, SysState] = ode45(@FinalProjStep2F, tspan, SysState0);

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
    xlabel('t');
    ylabel('theta and thetam');
    legend('theta', 'thetam');
    grid on;

    subplot(2,1,2);
    plot(t, x, t, xm);
    xlabel('t');
    ylabel('x and xm');
    legend('x','xm');
    grid on;
figure(2)
    subplot(2,1,1);
    plot(t, kx);
    xlabel('t');
    ylabel('kx');
    legend('kx1', 'kx2', 'kx3', 'kx4')
    grid on;

    subplot(2,1,2);
    plot(t, kr);
    xlabel('t');
    ylabel('kr');
    legend('kr1', 'kr2', 'kr3', 'kr4')
    grid on;

figure(3)
    subplot(2,1,1);
    plot(t, e);
    xlabel('t');
    ylabel('e');
    grid on;
    
    subplot(2,1,2);
    plot(1:100, EofT(length(EofT)-99:length(EofT)));
    xlabel('iter');
    ylabel('EofT');
    grid on;