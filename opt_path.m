
global h l k N alpha m m_gs
h=0.1; T=2; N=T/h; l=0.1; % h is the time-step, T is the total time and l is the distance between two wheels
m=4;                    %Rectangular domain of size m*
m_gs=1;					%This is the grid size. Smaller values means finer grid.
k=0.5; tol=1e-6; alpha=2; % k and alpha are parameters used in the objective function

g0=zeros(2*N,1); x0=zeros(2*N,1); H=eye(2*N); % Initializing states, gradient, Hessian values

j=1; %tt=0:h:T*h; % j is a variable used for loop count

x=zeros(N,5);  x(:,1:2)=1; % x represents the states here, i.e. x coordinate, y coordinate, angle of the robot and 
                           % the velocity inputs to each wheel

u(:,1)=0*randn(2*N,1); % Initializing inputs, (first input to the velocities

save_x=x;  % save_x is the the variable which actually has information about states and changes over time

[J0start,~]=marchandcomputefunction(u(:,1),x); % funeval is where the overall function is computed
J0=J0start;
j_check=0;

%% starting iteration
for kk=1:5000
    %function evaluation
    [J,save_x]=marchandcomputefunction(u(:,kk),save_x); 
%     if mod(kk,1000)==0
%         f=figure(1);plot(save_x(:,1),save_x(:,2),'-.*')
%         hold on
%         saveas(f,sprintf('path for 6X6 at %d.jpg',kk))
%         [~,~,phi]=compute_function_gradient(save_x(:,1),save_x(:,2),0,0);
%         f1=figure(2);contourf(phi);
%         saveas(f1,sprintf('contour for 6X6 at %d.jpg',kk))
%     end
    fun =  @(u) marchandcomputefunction(u,save_x);
    %gradient evaluation
    [gradient_lambda,save_x] = computegradient(save_x);
%     if mod(kk,100)==0
%         j=j+1;
%         g1_check=grad_check(u(:,kk),fun,1e-4);
%         compare_grad{j} = [g1_check [gradient_lambda(N+1:2*N);gradient_lambda(1:N)]];
%     end
        disp(strcat('iteration  =  ', num2str(kk), '  cost change wrt first  =   '  ,num2str(J0start-J), ' cost change wrt previous value  =  ',num2str(J0-J))) 

%% calculating descent direction p
    %getting direction p
    g1=[gradient_lambda(N+1:2*N);gradient_lambda(1:N)];
    g(:,kk)=g1;
    res=g1'*g1; res_save(kk)=res;
    if kk==1
        p=-g1/norm(g1); 
    else
        p=(-g1)+(res/res_old)*p;
        p=p/norm(p);
    end
    res_old=res;
    fun = @marchandcomputefunction; 
    a=1;

%% line search
      [a1]=armijo_condition(fun,J,[],u(:,kk),p,a,save_x);
     save_a(:,kk)=a1; % saving all backtracking coefficients
%% updating control input
    u(:,kk+1)=u(:,kk)+a1*p;
    if j_check==1
        J0=J0start;
    else
        J0=J;
    end
    J1(:,kk)=J;
    x0=save_x;
end
%%
close all; plot(save_x(:,1),save_x(:,2),'-.*')
    

    
    