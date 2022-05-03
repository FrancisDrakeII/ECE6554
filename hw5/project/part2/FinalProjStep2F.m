function SysStatedot = FinalProjStep2F(t,SysState)
disp(t);
%Set Params
global alpha1;
global alpha2;
global alpha3;
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
kr = SysState(13:16);
% alpha1 = SysState(17:36);
% alpha2 = SysState(37:56);
% alpha3 = SysState(57:76);
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
    0 0 0 L*sin(theta);
    0 0 0 -sin(theta)*cos(theta)];
D = [0;0; -g*gamma0*cos(theta)*sin(theta); M*g*sin(theta)*gamma0];

%Set gammas
%Note use bigger gamma to converge faster
gammax = 10; gammar = 10;
% gammaalpha = 10;
% ref, step signals here
r = [1;1;1;1];
%error
e= State-Statem;
%Place the poles
global p;
k =place(AL,BL,p);
Am = AL-BL*k;
P = lyap(Am',eye(4));
%Use Global! Or need to set center each time thus time consuming
global RBF;
RBF.statevec=State;
[Phi1,Phi2,Phi3] = Eval(RBF);
% %projection 
% alphadot = gammaalpha.*[Proj(alpha1,(e'*P*B).*Phi1);Proj(alpha2,(e'*P*B).*Phi2');Proj(alpha3,(e'*P*B).*Phi3)];
%Get fNN
fNN = alpha1*Phi1+ alpha2*Phi2'+ alpha3*Phi3;
% global EofT;
% Non = (sin(theta)*State(4).^2-M*g*tan(theta))/b;
% ET= Non-fNN;
% EofT = vertcat(EofT,abs(ET));
%U
u = kx'*State+kr'*r-fNN;
%Update
global Noise;
State = State+Noise;
Statemdot = Am*Statem+BL.*k'.*r;
Statedot = A*State + B*u +C*(State.*State)+D;
kxdot =-gammax.*State.*e.*sign(B);
krdot =-gammar.*r.*e.*sign(B);
SysStatedot = [Statedot;Statemdot;kxdot;krdot];
end