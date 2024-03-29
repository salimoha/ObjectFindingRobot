function [J,u,save_x]=snopt_for_unconstrained()
global h l k N alpha m m_gs
h=0.1; T=2; N=T/h; l=0.1; % h is the time-step, T is the total time and l is the distance between two wheels
m=4;                    %Rectangular domain of size m*
m_gs=0.25;					%This is the grid size. Smaller values means finer grid.
k=0.5; tol=1e-6; alpha=2; % k and alpha are parameters used in the objective function

g0=zeros(2*N,1); x0=zeros(2*N,1); H=eye(2*N); % Initializing states, gradient, Hessian values

j=1; %tt=0:h:T*h; % j is a variable used for loop count

x=zeros(N,5);  x(:,1:2)=1; % x represents the states here, i.e. x coordinate, y coordinate, angle of the robot and 
                           % the velocity inputs to each wheel

u(:,1)=randn(2*N,1); % Initializing inputs, (first input to the velocities

save_x=x;  % save_x is the the variable which actually has information about states and changes over time

[J0start,~]=marchandcomputefunction(u(:,1),x); % funeval is where the overall function is computed
J0=J0start;
j_check=0;
 fun =  @(u) marchandcomputefunction2(u,save_x);
% snsolve
[u] = solve_u(u, save_x);
[J,save_x]=marchandcomputefunction(u(:,1),x); 
close all; plot(save_x(:,1),save_x(:,2),'-.*')
    
end

function [x] = solve_u(x,save_x)
%% snopt configuration
global h l k N alpha m m_gs
snscreen off;
snprint('toymin.out');  % By default, screen output is off;

sntoy.spc = which('sntoy.spc');
% snspec (sntoy.spc);

% snseti ('Major Iteration limit', 250);
snseti ('Major Iteration limit', 2);

% x0=[0.4;0];
A1=full(gallery('tridiag',zeros(N-1,1),ones(N,1),ones(N-1,1)));

% % A=[blkdiag(A1,A1);[A1 -A1];[-A1 A1];blkdiag(-A1,-A1)]; %We need to make sure 
% % %that the difference does not go beyond our limit, which in essence states that 
% % %the velocity should not go above a certain value. That is why we need
% % %blkdiag(-A1,-A1)
% % b=[5*ones(2*N,1);(3.14/4)*ones(2*N,1);0*ones(2*N,1)];
A=[1 1 zeros(1,2*N-2)];
b=5;
Aeq    = [];
beq    = [];

lb     = -5*ones(2*N,1);
ub     = 5*ones(2*N,1);
%%
options.name = 'toyprob';
options.stop = @toySTOP;
 fun =  @(x) marchandcomputefunction2(x,save_x); 
[x,fval,INFO,lambda] = snsolve( fun, x, [], [], [], [], lb, ub, @consfungrad2, options);
snprint off;
snend;

end

function [c,ceq,DC,DCeq] = consfungrad2(x)
n = length(x);
c = [];
% No nonlinear equality constraints
ceq=[];
% Gradient of the constraints:
% % % if nargout > 2
DC = [];
   DCeq = [];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [iAbort] = toySTOP(itn, nMajor, nMinor, condZHZ, obj, merit, step, ...
			      primalInf, dualInf, maxViol, maxViolRel, ...
			      x, xlow, xupp, xmul, xstate, ...
			      F, Flow, Fupp, Fmul, Fstate)

% Called every major iteration
% Use iAbort to stop SNOPT (if iAbort == 0, continue; else stop)

iAbort = 0


end