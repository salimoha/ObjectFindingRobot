global h l k N alpha m m_gs
h=0.1; T=3; N=T/h; l=0.1; % h is the time-step, T is the total time and l is the distance between two wheels
m=5;                    %Rectangular domain of size m*
m_gs=0.25;
k=0.5; tol=1e-6; alpha=1; % k and alpha are parameters used in the objective function
x=zeros(N,3);  x(:,1:2)=1; u(:,1)=1*randn(2*N,1); save_x=x;  

% the variables include control inputs and the velocities using which turn
% rate is defined

    [J0start,~]=marchandcomputefunction(u(:,1),x); % funeval is where the overall function is computed
    fun =  @(u) marchandcomputefunction(u,save_x);
    pp=fmincon(fun,u,[],[],[],[],[0.5*ones(N,1);-3.14*ones(N,1)],[3*ones(1*N,1);3.14*ones(N,1)]);
    [J,save_x]=marchandcomputefunction(pp,save_x);

%     pp = reshape (pp,N,2);
    
figure, plot(save_x(:,1),save_x(:,2),'-.*'),title('Result using fmincon');
