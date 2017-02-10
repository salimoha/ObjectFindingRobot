function dx=robot_system_equations(x,u)
global l
% actual system as function of time
% see Lumond
dx(1)=(x(4)+x(5))*cos(x(3))*0.5;
dx(2)=(x(4)+x(5))*sin(x(3))*0.5;
dx(3)=(1/l)*(x(4)-x(5));
dx(4)=u(1);
dx(5)=u(2);
end

%linearised A=[0 0 -x(4)+x(5))*sin(x(3))*0.5 0.5*cos(x(3)) 0.5*cos(x(3));0 0 (x(4)+x(5))*cos(x(3))*0.5 0.5*sin(x(3)) 0.5*sin(x(3));0 0 0 1/l -1/l;0 0 0 0 0;0 0 0 0 0];
%B=[0 0 0 1 0;0 0 0 0 1]';