function [pp,J]=output_lrhb(u)
global h l k N alpha m m_gs
h=0.1; T=2; N=T/h; l=0.1; % h is the time-step, T is the total time and l is the distance between two wheels
m=4;                    %Rectangular domain of size m*
m_gs=0.25;
k=0.5; tol=1e-6; alpha=2; % k and alpha are parameters used in the objective function
x=zeros(N,3);  x(:,1:2)=1; u(:,1)=1*randn(2*N,1); save_x=x;  

tic

x=zeros(N,3);  x(:,1:2)=1; save_x=x; 
u=1*randn(2*N,1);  

% the variables include control inputs and the velocities using which turn
% rate is defined

lb=[0.5*ones(N,1);-100*ones(N,1)]; ub=[3*ones(N,1);100*ones(N,1)];

fun =  @(u) marchandcomputefunction2(u,save_x);
[pp,itn,fnum,flag] = LRHBe16D( fun, u, lb, ub );

[J,save_x]=marchandcomputefunction(pp,save_x);

figure, plot(save_x(:,2),save_x(:,1),'-.*'),title('Result using lrhb');
toc