function dx=robot_system_equations(x,u)
global l %x=[qx;qy;theta] u=[v w];
% actual system as function of time
% see Lumond
dx(1)=u(1)*cos(x(3));
dx(2)=u(1)*sin(x(3));
dx(3)=u(2);
end

%linearised A=[0 0 -x(4)+x(5))*sin(x(3))*0.5 0.5*cos(x(3)) 0.5*cos(x(3));0 0 (x(4)+x(5))*cos(x(3))*0.5 0.5*sin(x(3)) 0.5*sin(x(3));0 0 0 1/l -1/l;0 0 0 0 0;0 0 0 0 0];
%B=[0 0 0 1 0;0 0 0 0 1]';