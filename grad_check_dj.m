function [ grad_numeric] = grad_check_dj(x,fun,delta,p)
% calculates the finite difference gradiant to check the gradient formula
 n = length(x);
%  keyboard
err=0;
% for ii=1:n
%     Delta_k = [zeros(ii-1,1) ; 1; zeros(n-ii,1)];
    Delta_k = zeros(n,1);
    Delta_k(p)=1;
     grad_numeric(1) = (fun(x(:,1)+(Delta_k.*delta),x(:,2)) - fun(x(:,1)-(Delta_k.*delta),x(:,2)))/(2*delta);
     grad_numeric(2) = (fun(x(:,1),x(:,2)+(Delta_k.*delta)) - fun(x(:,1),x(:,2)-(Delta_k.*delta)))/(2*delta);
%    grad_numeric(ii) = (fun(x(ii)+delta) - fun(x(ii)))./delta;
%     grad_theory(ii) = grad(x(ii));
%     err = err + ( grad_theory(ii) -  grad_numeric(ii,ii))^2;
% end


end

