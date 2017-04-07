n=20; nCon=1; b=[]; beq=[]; mi=[]; nli=[]; nle=[];
xmul    =  zeros(n,1);
xstate  =  zeros(n,1);
Fmul    =  zeros(nCon,1);
Fstate  =  zeros(nCon,1);
ObjAdd  =  0;
ObjRow  =  1;
Flow    = [ -inf; -inf*ones(nli,1); zeros(nle,1); -inf*ones(mi,1); beq ];
Fupp    = [  inf; zeros(nli,1); zeros(nle,1); b; beq ];

global h l k N alpha m m_gs
h=0.1; T=1; N=T/h; l=0.1; % h is the time-step, T is the total time and l is the distance between two wheels
m=4;                    %Rectangular domain of size m*
m_gs=0.25;
k=0.5; tol=1e-6; alpha=2; % k and alpha are parameters used in the objective function
g0=zeros(2*N,1); x0=zeros(2*N,1); H=eye(2*N); % Initializing states, gradient, Hessian values
j=1; %tt=0:h:T*h; % j is a variable used for loop count
x=zeros(N,5);  x(:,1:2)=1; % x represents the states here, i.e. x coordinate, y coordinate, angle of the robot and 
                           % the velocity inputs to each wheel
u(:,1)=1*randn(2*N,1); % Initializing inputs, (first input to the velocities
save_x=x;  % save_x is the the variable which actually has information about states and changes over time
fun = @(u) marchandcomputefunction2(u,save_x);

u=snsolve(fun,u,eye(20),ones(20,1));
