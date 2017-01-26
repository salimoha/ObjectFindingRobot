function [a]=armijo_condition(fun,f1,g1,u,p,a,x)    
gamma=0.9;%a=1;
eta=0.5;
% %armijo condition
[f2]=fun(u+a*p,x);

    while f2>f1%+eta*a*g1'*p,
        a=gamma*a;
        if norm(a*p)<1e-6
%             a=0;
            break
        end
        [f2]=fun(u+a*p,x);
    end
  %  keyboard
    end
