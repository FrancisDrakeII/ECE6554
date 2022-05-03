% x(1) = position
% x(2) = velocity
%
% xdot(1) = velocity = x(2)
% xdot(2) = acc

function xdot = f(~, x)
u = heaviside(sym(1));

K = 0.567;
Kd = 0.625;

xdot = zeros(2,1); 

xdot(1) = x(2);    
xdot(2) = u - 2*K*x(1) + (-2*Kd+0.5)* x(2);
end