% plant model
a = 2; b = 3/2;
% reference model
am = -2; bm = 3;
% estimated plant model
a_hat = 1; b_hat = 2;

% initialized xm, x, kx, kr
xm_0 = 0;
x_0 = 0;
kx_0 = (am-a_hat)/b_hat;
kr_0 = bm/b_hat;
%ideal
kx_s = (am-a)/b;
kr_s = bm/b;

sys_states_0 = [x_0, xm_0, kx_0, kr_0];

% time
tmax = 50;
tspan = [0, tmax];


[t, sys_states] = ode45(@q7_ode, tspan, sys_states_0);


x = sys_states(:,1);
xm = sys_states(:,2);
kx = sys_states(:,3);
kr = sys_states(:,4);
delta_kx = kx-kx_s;
delta_kr = kr-kr_s;

figure(1);
    subplot(3,1,1);
    plot(t, delta_kx+abs(am)/bm*delta_kr);
    xlabel('time(s)');
    ylabel('states');
    title('x, xm with gamma_x = 1e-3 , gamma_r = 1e-3, r(t) = 5')
    legend('z(t)');
    grid on;

    subplot(3,1,2);
    plot(t, kx);
    xlabel('time(s)');
    title('Kx')
    grid on;

    subplot(3,1,3);
    plot(t, kr);
    xlabel('time(s)');
    title('Kr')
    grid on;
