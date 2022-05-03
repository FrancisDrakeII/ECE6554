%Set param
tspan = [0, 20];

x0 = [0;0.1;0;0];

[t, x] = ode45(@HW5Q7FLNL, tspan, x0);


figure(1);
    subplot(2,1,1);
    plot(t, x(:,2));
    xlabel('t');
    ylabel('theta');
    title('NonLinearSys+Perturb+Linear Controller: theta')  
    grid on;

    subplot(2,1,2);
    plot(t, x(:,1));
    xlabel('t');
    ylabel('trajectory x');
    title('NonLinearSys+Perturb+Linear Controller: x')  
    grid on;
