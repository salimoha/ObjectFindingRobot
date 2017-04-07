function [J,x]=theta_as_constraint_funeval(u,x) % function to evaluate the value of objective function 
                               % and also march the states x using RK4
                               % inputs are "control input and states"
                               % outputs are value of objective function J,
                               % states x
global h k N alpha

x=[x u(2*N+1:3*N) u(3*N+1:end)];
u=u(1:2*N);
u=reshape(u,N,2);

for ii=1:N

    fun_dynamic=@(x) robot_system_equations(x,u(ii,:)); % vehicle time march is obtained from Laumond
    x(ii+1,:)=robot_forward_march(x(ii,:),fun_dynamic,h); % note the states are marched one step extra than the input
    
end

J=compute_function_gradient(x(:,1),x(:,2),0);   % firstdiff is the function where the actual value of the objective function is computed
