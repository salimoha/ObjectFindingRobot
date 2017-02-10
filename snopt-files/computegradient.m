function [gradient,x]=computegradient(x) % function to compute the gradient, the input
                              % is the states and the outputs are gradient
                              % and the states
                              % NOTE: although the states are not supposed
                              % to change in this function, the states that
                              % is read in by gfun will have one row extra,
                              % this will be corrected in this function
global N h

lmd=zeros(N+1,5);  % Initializing the adjoint variable to be the same as states
lmd(N+1,:)=[0 0 0 0 0]; 

  for i=N+1:-1:2
    adj =@(lmd,i) adjoint(lmd,i,x);         % adjoint is the function where the adjoint variables are calculated
    lmd(i-1,:)=adjoint_backward_march(lmd(i,:),adj,-h,i);   % adjoint is marched backward in time using RK4
  end

    B=[0 0 0 1 0;0 0 0 0 1]';  % B is the input matrix
    gradient=lmd*B;            % gradient for the objective function is computed using adjoint varible
    gradient(end,:)=[];
    gradient=reshape(gradient,2*N,1);
    x(end,:)=[];
end

