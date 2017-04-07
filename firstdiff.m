function [big_phi,grad_phi]=firstdiff(qx,qy,pp,p) % this is the function where 
                                            % the actual computation of
                                            % objective function are dJ/dx
                                            % takes place
global k N m m_gs
n=size(qx);
phi=1;
big_phi=0;
grad_phi=zeros(2,1);

% if we just want to evaluate value of objective 
for ii=1:m_gs:m
        for jj=1:m_gs:m
            for kk=1:n
            phi=phi*function_eval(qx(kk),qy(kk),ii,jj); % phi represents the value of objective function over each point in the domain
            % function_eval contains the expression of the objective
            % function
            end
            big_phi=big_phi+phi^2;
        end
        phi=1;
end
%     big_phi = phi.^2;
    big_phi=sum(big_phi(:)); % value of objective function is computed as a summation of all values over the domain
if pp==1  % Condition if we want to evaluate gradient dJ/dx for adjoint calculation
    phi=1;
    for i=1:m_gs:m
        for j=1:m_gs:m
            for kk=1:n
                if kk==p 
                    continue; % objective function is not computed for qx(p) and qy(p), as these values will be differentiated and obtained as a
                              % 2*1 vector from gradient_eval
                else
                    phi = phi * function_eval(qx(kk),qy(kk),i,j);
                end
            end
            dphi = gradient_eval(qx(p),qy(p),i,j); %gradient_eval contains the expression for gradient of the objective function
            grad_phi = grad_phi + dphi*(phi^2);
            phi=1;
        end
    end
end
  
if pp~=0
    grad_phi=[grad_phi;0]; % Since states contain 5 variables, the gradient must be a 5*1 vector of which only the first 2 will have values
                               % and other 3 are 0 as the objective
                               % function does not depend on theta and the
                               % velocities, it only depends on the
                               % position of the robot
end

end
