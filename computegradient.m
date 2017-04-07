function [gradient,x]=computegradient(x,u) % function to compute the gradient, the input
                              % is the states and the outputs are gradient
                              % and the states
                              % NOTE: although the states are not supposed
                              % to change in this function, the states that
                              % is read in by gfun will have one row extra,
                              % this will be corrected in this function
global N h
u=reshape(u,N,2);
u(end+1,:)=u(end,:);
lmd=zeros(N+1,3);  % Initializing the adjoint variable to be the same as states
lmd(N+1,:)=[0 0 0]; 
gradient=zeros(10,2);
  for i=N+1:-1:2
    adj =@(lmd,i) adjoint(lmd,i,x,u(i,:));         % adjoint is the function where the adjoint variables are calculated
    lmd(i-1,:)=adjoint_backward_march(lmd(i,:),adj,-h,i);   % adjoint is marched backward in time using RK4
    gradient(i-1,:)=lmd(i-1,:)*[cos(x(i-1,3)) 0;...
                            sin(x(i-1,3)) 0;...
                                    0 1];
  end

%     B=[0 0 0 1 0;0 0 0 0 1]';  % B is the input matrix
%     gradient=lmd*B;            % gradient for the objective function is computed using adjoint varible
%     gradient(end,:)=[];
    gradient=reshape(gradient,2*N,1);
    x(end,:)=[];
end