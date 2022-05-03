function SysStatedot = FinalProjStep3F(t,SysState)
%Set Params
disp(t);
mc =0.5;
mp =0.1;
Jp = 0.006;
l = 0.3;
deltatheta = 0.005;
deltax = 0.01;
g =9.8;
M = (mc+mp)/mp/l;
L = Jp/mp/l;
b = 1/mp/l;
Dx = deltax/mp/l;
Dtheta = deltatheta/mp/l;
%Get State out
State = SysState(1:4);
Statem = SysState(5:8);
kx = SysState(9:12);
kr = SysState(13);
W = SysState(14);
Phi = SysState(15);

theta = State(2);
gamma0 = 1/(M*L-cos(theta).^2);
%Find control matrix
A = [0 0 1 0;
    0 0 0 1;
    0 0 -L*Dx*gamma0 Dtheta*cos(theta)*gamma0;
    0 0 Dx*cos(theta)*gamma0 -M*Dtheta*gamma0];
%A-linear, need pole placement to get the Am
AL = [0 0 1 0;
    0 0 0 1;
    0 -g/(M*L-1) -L*Dx/(M*L-1) Dtheta/(M*L-1);
    0 M*g/(M*L-1) Dx/(M*L-1) -M*Dtheta/(M*L-1)];

B = [0; 0; b*L*gamma0;-b*cos(theta)*gamma0];

BL = [0; 0; b*L/(M*L-1);-b/(M*L-1)];
%Non-linear term C*State^2 and D
C = [0 0 0 0;
    0 0 0 0;
    0 0 0 L*sin(theta)*gamma0;
    0 0 0 -sin(theta)*cos(theta)*gamma0];

D = [0;0; -g*gamma0*cos(theta)*sin(theta); M*g*sin(theta)*gamma0];

%Set gammas
%Note use bigger gamma to converge faster
gammax = 10; gammar = 10;gammaw = 10;gammaphi =10;

% ref, step signals here
r = 1;
%error
e= State-Statem;
%Place the poles
global p;
k =place(AL,BL,p);
Am = AL-BL*k;
P = lyap(Am',eye(4));
%U
Phix = sin(theta)/b*State(4)*State(4);
% disp(Phix);
u = kx'*State+kr*r-W*Phix-Phi*tanh(e'*P*B/20);
%Update
% global Noise;
% State = State+Noise;
Statemdot = Am*Statem+BL.*k'.*r;
Statedot = A*State + B*u +C*State.*State+D;
kxdot =gammax.*Projfunc(kx,-State*e'*P*B);
krdot =gammar.*Projfunc(kr,-r*e'*P*B);
% disp(Phix*e'*P*B);
Wdot = gammaw.*Projfunc(W,Phix*e'*P*B);
Phidot = gammaphi.*Projfunc(Phi,e'*P*B*tanh(e'*P*B/20));
SysStatedot = [Statedot;Statemdot;kxdot;krdot;Wdot;Phidot];
end