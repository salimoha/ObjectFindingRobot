function dphi=gradient_eval(qx,qy,i,j) % gradient is computed by differentiating
                                     % the relevant part of objective
                                     % function, inputs are qx(p) and
                                     % qy(p), i.e. the position of the
                                     % robot at whih the gradient is
                                     % computed and i,j which denotes the
                                     % point on the domain

global k alpha

dphi_x=(2*(1 - k * exp( -alpha * ((i-qx)^2 + (j-qy)^2) ) ) ) * (-k * exp( -alpha * ((i-qx)^2 + (j-qy)^2) )) * 2*alpha*(i - qx);
dphi_y=(2*(1 - k * exp( -alpha * ((i-qx)^2 + (j-qy)^2) ) ) ) * (-k * exp( -alpha * ((i-qx)^2 + (j-qy)^2) )) * 2*alpha*(j - qy);

dphi = [dphi_x;dphi_y];