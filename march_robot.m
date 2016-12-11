function [x]=march_robot(u,x) % function to evaluate the value of objective function 
                               % and also march the states x using RK4
                               % inputs are "control input and states"
                               % outputs are value of objective function J,
                               % states x
global h k N alpha

N=length(u)/2;
u=reshape(u,N,2);
    
for ii=1:N-1

    fun_dynamic=@(x) vehicle_time_march(x,u(ii,:)); % vehicle time march is obtained from Laumond
    x(ii+1,:)=RK4_timemarch(x(ii,:),fun_dynamic,h); % note the states are marched one step extra than the input
    
end
  