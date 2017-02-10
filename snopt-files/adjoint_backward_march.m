function lmd1=adjoint_backward_march(lmd,adj,h,i)

f1=adj(lmd,i);
f2=adj(lmd+h/2*f1',i);
f3=adj(lmd+h/2*f2',i);
f4=adj(lmd+h*f3',i);
lmd1=lmd+h/6*(f1'+2*f2'+2*f3'+f4');

end