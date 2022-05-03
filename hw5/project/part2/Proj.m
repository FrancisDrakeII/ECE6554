function pro = Proj(y,theta)
%This is the Projection function for Step2
%Param setteing
%ep is 1e-2 & max is 2;
     pro = zeros(20,1);
     f = -bsxfun(@minus,2,y.^2)*25;
     grad = 100*y;
     for i=1:20
        if (f(i)>0 && transpose(theta(i))*grad(i)<0)
            pro(i) = theta(i)-((grad(i)*transpose(grad(i)))*theta(i)*f(i))/norm(grad(i));
        else
            pro(i) = theta(i);
        end
     end
end

