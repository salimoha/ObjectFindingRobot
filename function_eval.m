function [phi]=function_eval(qx,qy,i,j)
% let qx and qy be the position of the robot
global k alpha

% for i=1:10
%     for j=1:10
%         phi = sum( [ phi, (1 - k*exp( -alpha * ((i - qx)^2 + (j - qy)^2) ) ) ] );
% % phi(i,j)=1 - k*exp( -alpha * ((i - qx)^2 + (j - qy)^2) );
%     end
% end
phi = 1 - k*exp( -alpha * ((i - qx)^2 + (j - qy)^2) ) ;