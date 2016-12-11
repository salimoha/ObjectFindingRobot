function value=adjoint(lmd,p,x) % adjoint computes the 5*1 adjoint variable, 
                    % for each step. Inputs are initialized lmd, p - for
                    % relevant operation to obtain gradient dJ/dx from
                    % firstdiff and the states x

global l
%linearised model of dynamics
A=[0 0 -(x(p,4)+x(p,5))*sin(x(p,3))*0.5 0.5*cos(x(p,3)) 0.5*cos(x(p,3));...
    0 0 (x(p,4)+x(p,5))*cos(x(p,3))*0.5 0.5*sin(x(p,3)) 0.5*sin(x(p,3));...
    0 0 0 1/l -1/l;...
    0 0 0 0 0;...
    0 0 0 0 0];

%get objective function and first differential
qx=x(:,1);qy=x(:,2);

[~,dj]=compute_function_gradient(qx,qy,1,p); % dJ/dx is computed here
% fun = @(qx,qy) firstdiff(qx,qy,0,p);
% dj_check=grad_check_dj([qx,qy],fun,1e-4,p);

    value =  dj + (lmd*(A))';   %+ll*eye(size(A))))'; %check the signs, no - before dj previously

end