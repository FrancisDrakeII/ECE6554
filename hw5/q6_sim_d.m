A = [1, 3; -1, 2];
B = [0; 2];
Q = [4.5, 2.8; 2.8, 2.5];

x0 = [1.5; 2];

[X, L, G] = care(A, B, Q);

Am = A - B*G;
%P = lyap(Am, Q);
P = X;

global x_dot_matrix;
x_dot_matrix = zeros(999,2);
global cnt1;
cnt1 = 1;

global x_lqr_dot_matrix;
x_lqr_dot_matrix = zeros(999,2);
global cnt2;
cnt2 = 1;

global epsilon
epsilon = 1.5;

sys_init = [x0(1); x0(2); 0; 0; 0; 0];
tspan = [0,20];
[t,states] = ode45(@q6_fun_d, tspan, sys_init);
x = states(:,1:2);

[t_lqr, lqr_states] = ode45(@q6_fun_LQR_d, tspan, sys_init);
x_lqr = lqr_states(:,1:2);

V = zeros(size(x, 1),1);
Vdot = zeros(size(x, 1),1);

V_lqr = zeros(size(x_lqr, 1),1);
Vdot_lqr = zeros(size(x_lqr, 1),1);



for i =1:length(t)
    %Get gains
    fai0 =  x(i,:)*(A'*P+P*A)* x(i,:)'+epsilon* x(i,:)*P* x(i,:)';
    fai1 = 2* x(i,:)*P*B;
    if (fai0>0) 
        u = -fai0*fai1/fai1'/fai1;
    else
        u =0;
    end
    Vdot(i) = x(i,:)*(A'*P+P*A)*x(i,:)'+2*x(i,:)*P*B*u;
end

for j = 1:(size(V_lqr,1))
    V_lqr(j) = x_lqr(j,:)*P*transpose(x_lqr(j,:));
    Vdot_lqr(j) = 2*x_lqr(j,:)*P*transpose(x_lqr_dot_matrix(j,:));
end


figure(1);
%     subplot(2,1,1);
    plot(t, Vdot);
    xlabel('Time(s)');
    grid on;
    title('Vdot epsilon = 1.5');
% 
%     subplot(2,1,2);
%     plot(t_lqr, Vdot_lqr);
%     xlabel('Time(s)');
%     grid on;
%     title('V dot LQR');
    