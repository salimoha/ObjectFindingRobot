fun =  @(u) marchandcomputefunction(u,save_x);
pp=fmincon(fun,zeros(40,1),[],[],[],[],-7*ones(40,1),7*ones(40,1));
[J,save_x]=marchandcomputefunction(pp,save_x);

figure, plot(save_x(:,1),save_x(:,2),'-.*'),title('Result using fmincon');
[~,~,phi]=compute_function_gradient(save_x(:,1),save_x(:,2),0,0);
        f1=figure(2);contourf(phi);