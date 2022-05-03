classdef RBFclass
%The RBFclass used for step2
    properties
     %center matrix:
     CmatrixX
     CmatrixY
     CmatrixZ
     statevec
     sigmavec
    end
    methods
        function [Phi1,Phi2,Phi3] = Eval(obj)
            %Evaluating the Phi s
            %OK, first calculate the Norm of each variable. Since I use
            %20*20*20 neurons, thus it will be a 20*20*20 cube
            Normx = bsxfun(@minus,obj.statevec(2),obj.CmatrixX).^2;
            Normy = bsxfun(@minus,obj.statevec(3),obj.CmatrixY).^2;
            Normz = bsxfun(@minus,obj.statevec(4),obj.CmatrixZ).^2;
            %Sum them
            Norm = Normx+Normy+Normz; 
            %Find RBF matrix
            RBFmatrix1 = exp(-Norm.*obj.sigmavec(1));
            RBFmatrix2 = exp(-Norm.*obj.sigmavec(2));
            RBFmatrix3 = exp(-Norm.*obj.sigmavec(3));
            %Sum over different dims, and get the final results
            Phi1 = sum(RBFmatrix1,[2,3]); %sum over RBFmatrix1's 2nd and 3rd dimension
            Phi2 = sum(RBFmatrix2,[1,3]); %1st and 3rd
            Phi3 = sum(RBFmatrix3,[1,2]); %1st and 2nd
            %need reshape since matlab sum's property. Won't affect the
            %final result
            Phi3 = reshape(Phi3,[20,1]);
        end
    end
end
