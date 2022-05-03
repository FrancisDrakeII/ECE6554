% Plant
a = 5; b = 3;
% Reference
am = -4; bm = 4;
% Estimated plant
a_hat = 1; b_hat = 2;

% Initialization
xm_0 = 0;
x_0 = 0;
kx_0 = (am-a_hat)/b_hat;
kr_0 = bm/b_hat;
ku_0 = 0;

sys_states_0 = [x_0, xm_0, kx_0, kr_0, ku_0];

% Time
tmax = 2;
tspan = [0, tmax];

% Simulation
[t, sys_states] = ode45(@q3_ode, tspan, sys_states_0);

% Taking out the sys_states
x = sys_states(:,1);
xm = sys_states(:,2);
kx = sys_states(:,3);
kr = sys_states(:,4);
ku = sys_states(:,5);

e = x - xm;

% Reference i/p:
r = 4*sin(2*t);
% r =5;
uc = kx.*x + kr.*r;

u_max = 9;


u  = zeros(size(uc,1));
for i =1:size(uc,1)
    if abs(uc(i))< u_max
        u(i) = uc(i);
    else
        u(i) = u_max*sign(uc(i));
    end
end




figure(1);
    subplot(5,1,1);
    plot(t, x, t, xm);
    xlabel('Time(sec)');
    ylabel('States');
    title('x and xm with gamma_x = 0.1, gamma_r = 0.1, r(t) = 4sin(2t)')
    grid on;

    subplot(5,1,2);
    plot(t, e);
    xlabel('Time(sec)');
    title('error');
    grid on;
    
    subplot(5,1,3);
    plot(t, kx);
    xlabel('Time(sec)');
    title('Kx')
    grid on;

    subplot(5,1,4);
    plot(t, kr);
    xlabel('Time(sec)');
    title('Kr')
    grid on;

    subplot(5,1,5);
    plot(t, uc, t, u);
    xlabel('Time(sec)');
    title('uc, u');
    grid on;
    legend('uc','u');
