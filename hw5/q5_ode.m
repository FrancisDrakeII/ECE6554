function sys_states_dot = f(t, sys_states) 

% Plant
a = 2; 
b = 2;
% Reference
am = -2; 
bm = 3;
% Adaptive control gains
global gamma_a; global gamma_b;
gamma_a = 20; 
gamma_b = 20;

b_bar = 0.75;
epsilon = 0.001;

% Reference i/p:
% r = 5;
r = 4*sin(3*t);
% Taking out the states accordingly
x  = sys_states(1);
xm = sys_states(2);
a_hat = sys_states(3);
b_hat = sys_states(4);
% error
e = x - xm;

 

%indirect adaptive feedback u(t)
u = (1/b_hat)*(-a_hat*x+am*x+bm*r);


% Updating the state variables

x_dot = a*x+b*u;
xm_dot = am*xm + bm*r;
a_hat_dot = gamma_a*x*e;
if b_hat >= b_bar
    b_hat_dot = gamma_b*u*e;
else
    b_hat_dot = gamma_b*u+ (b_bar-b_hat)/(b_hat-b_bar+epsilon);
end

% Putting the states together and return
sys_states_dot = [x_dot; xm_dot; a_hat_dot; b_hat_dot];

end

