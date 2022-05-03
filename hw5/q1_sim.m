% Adaptive control gains
global gamma_x;
global gamma_r;
gamma_x = 0.1; 
gamma_r = 0.1;
% reference model
am = -2; 
bm = 3;
% estimated plant
a_hat = 1; 
b_hat = 2;
% initialization
xm0 = 0;
x0  = 0;
kx0 = 0;
kr0 = 0;
system_states_init = [x0, xm0, kx0, kr0];
% setting time
tmax = 20;
T=0:0.001:tmax;
tspan = [0, tmax];
STD = 0.25;
global Noise_1; 
Noise_1 = STD*randn(1,length(T));
global Noise_2; 
Noise_2 = STD*randn(1,length(T));
global Noise_3; 
Noise_3 = STD*randn(1,length(T));
global Noise_4; 
% global r;
Noise_4 = STD*randn(1,length(T));
% stimulation
[t, system_states] = ...
    ode45(@q1_ode, tspan, system_states_init);
% system_states
x  = system_states(:,1);
xm = system_states(:,2);
kx = system_states(:,3);
kr = system_states(:,4);
figure(1);
    subplot(3,1,1);
    plot(t, x, t, xm);
    xlabel('Time(sec)');
    ylabel('States');
    title(['x and xm with \gamma_x = ',num2str(gamma_x),' \gamma_r = ',num2str(gamma_x),', STD = ',num2str(STD),', r(t) = 4sin(3t)'])
    legend('x', 'x_m');
    grid on;
    
    subplot(3,1,2);
    plot(t, kx);
    xlabel('Time(sec)');
    title('Kx')
    grid on;
    
    subplot(3,1,3);
    plot(t, kr);
    xlabel('Time(sec)');
    title('Kr')
    grid on;
