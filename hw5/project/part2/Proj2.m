function pro = Proj2(y,theta)
%This is the Projection function for Step2
%Param setteing
%ep is 1e-2 & max is 2;
     pro = zeros(20,1);
     f = -bsxfun(@minus,2,y.^2)*2.5;
     grad = 10*y;
     for i=1:20
        if (f(i)>0 && theta(i)*grad(i)<0)
            pro(i) = theta(i)+theta(i)*f(i);
        else
            pro(i) = theta(i);
        end
     end
end