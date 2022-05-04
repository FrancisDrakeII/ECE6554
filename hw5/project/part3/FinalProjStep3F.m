function SysStatedot = FinalProjStep3F(t,SysState)
%Set Params
disp(t);
mc =0.55;
mp =0.1;
Jp = 0.006;
l = 0.3;
deltatheta = 0.05;
deltax = 0.09;
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
kr = SysState(13:16);
W = SysState(17);
Phi = SysState(18);

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
r = [sin(t);0;0;0];
%error
e= State-Statem;
%Place the poles
global p;
k =place(AL,BL,p);
Am = AL-BL*k;

Q =eye(4);
P = lyap(Am',Q);
Ind = e'*P*B;
%U
Phix = sin(theta)*State(4)*State(4);

u = kx'*State+kr'*r-W*Phix-Phi*tanh(Ind/10);
%Update
%Noise;
%UIUC lecture
Statemdot = Am*Statem+BL.*k'.*r;
Statedot = A*State + B*u +C*State.*State+D;
% Statedot = A*(State+0.25*rand(4,1))+ B*u +C*(State+0.25*rand(4,1)).*(State+0.25*rand(4,1))+D;
kxdot =gammax.*Projfunc3(kx,-State.*Ind);
krdot =gammar.*Projfunc3(kr,-r*Ind);
Wdot = gammaw.*Projfunc3(W,Phix*Ind);
Phidot = gammaphi.*Projfunc3(Phi,Ind*tanh(Ind/10));
SysStatedot = [Statedot;Statemdot;kxdot;krdot;Wdot;Phidot];
end