function proj = Projfunc3(theta,y)
thetamax = 10;
ep=0.01;
n=length(theta);
proj = zeros(n,1);
if n>1
    for i =1:n
        f = (theta(i)*theta(i)-thetamax*thetamax)/ep/thetamax/thetamax;
        grad = 2*theta(i)/ep/thetamax;
        if f>0 && y(i)*grad<0
            proj(i) = y(i)+y(i)*f;
        else
            proj(i) = y(i);
        end
    end
else
        f = (theta*theta-thetamax*thetamax)/ep/thetamax/thetamax;
        grad = 2*theta/ep/thetamax;

        if f>0 && y*grad<0
            proj = y+y*f;
        else
            proj = y;
        end
end
% disp('++++++++');
% disp(proj);
% disp('============');
end