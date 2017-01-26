function x1=robot_forward_march(x,fun,h)

f1=fun(x);
f2=fun(x+h/2*f1);
f3=fun(x+h/2*f2);
f4=fun(x+h*f3);
x1=x+h/6*(f1+2*f2+2*f3+f4);
end