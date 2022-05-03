function sys_states_dot = f(t, sys_states) 

sys_states = transpose(sys_states);
%disp(sys_states)

K = 0.567;
Kd = 0.625;

% Reference
am = [0, 1; -2*K, -2*Kd+0.5]; bm = [0;1];
% Plant
a = [0,1;0, 0.5]; b = [0;2];

% Adaptive control gains
gamma_x = .1; gamma_r = .1;

% Reference i/p:
r = 1;         %(c)
%r = sin(t);    %(d)

% Taking out the states accordingly

x = transpose(sys_states(1:2));
xm = transpose(sys_states(3:4));
kx = transpose(sys_states(5:6));
kr = sys_states(7);

e = x - xm;
Q = [1, 0; 0, 1];
P = lyap(am,Q);

tmax = 20;
T=0:0.001:tmax;
global Noise_1; 
%noise_1 = interp1(T,Noise_1,t,'nearest');
noise_1 = 0;

sigma_x = 0.01;
sigma_r = 0.01;

% Updating the state variables
% a = 2x2, b = kx = 2x1 , x = 2x1 , kr = 1x1, r= 1x1
x_dot = (a + b.*kx)*(x + noise_1*[1;1])+ b.*kr.*r;
xm_dot = am*xm + bm*r;
kx_dot = transpose(-gamma_x*transpose(b)*P*e*transpose(x+ noise_1*[1;1])-gamma_x*sigma_x*transpose(kx)); % .*sign(b);
kr_dot = -gamma_r*transpose(b) *P*e*r - sigma_r*kr; % .*sign(b);

% Putting the states together and return
sys_states_dot = [(x_dot); (xm_dot); (kx_dot); kr_dot];

end
