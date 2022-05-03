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
xm0 = 0;x0  = 0;kx0 = 0;kr0 = 0;psi0 = 0;
system_states_init = [x0, xm0, kx0, kr0, psi0];

global tmax;
tmax = 50;
T=0:0.01:tmax;
tspan = [0, tmax];

% stimulation
[t, system_states] = ode45(@q1_ode, tspan, system_states_init);
% system_states
x  = system_states(:,1);
xm = system_states(:,2);
kx = system_states(:,3);
kr = system_states(:,4);
psi = system_states(:,5);
e = x-xm;

figure(1);
    subplot(4,1,1);
    plot(t, x, t, xm);
    xlabel('time(s)');
    ylabel('States');
    title(['x and xm with \gamma_x = ',num2str(gamma_x),' \gamma_r = ',num2str(gamma_x),' r(t) = 0.25cos(2t) noise = 3cos(3t) disturbance = 3cos(t)'])
    legend('x', 'x_m');
    grid on;
    
    subplot(4,1,2);
    plot(t, kx);
    xlabel('time(s)');
    title('Kx')
    grid on;
    
    subplot(4,1,3);
    plot(t, kr);
    xlabel('time(s)');
    title('Kr')
    grid on;

    subplot(4,1,4);
    plot(t, e);
    xlabel('time(s)');
    title('error')
    grid on;
