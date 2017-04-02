import numpy as np
import pylab as plt

from Utils import modichol
from libVehicle import funeval, gfun, armijo_condition

h = 0.1
T = 1
N = T/h
l = 10
lmd = np.zeros((N,5))
xstar = [1, 1]
k = 0.01
tol = 1e-6
J0 = 0
g0 = np.zeros((2*N,1))
x0 = np.zeros((2*N,1))
H = np.eye((2*N))


tt = np.arange(0,T*h, h)
x = [1, 1, 0, 0, 0]

NN = 1000
METHOD_ORDER = 1
u = np.zeros((2*N,1))
g = np.zeros((2*N,NN)) #dont know the right size
J1 = np.zeros((2*N,NN)) #dont know the right size
y = np.zeros((11,5)) #dont know the right size
u[:,0] = np.random.randn(2*N)
p = 0

for kk in range(0,NN):
    pp = 1
    # function evaluation
    [J,x] = funeval(u[:,kk], h, N)
    [gradient,x] = gfun(x, N, h)
    print("iteration = %d " %kk + "   J(x) = %d" %J)
    if (kk==0):
        ax0 = plt.figure()
        ax0.plot(x[:,0], x[:,2],'k-')
        pp = 5

    g[:,kk] = gradient
    if (J<tol):
        break

    #getting direction p
    g1 = gradient
    x1 = x[:,0:1]
    x1 = np.reshape(x1, 2*N, 1)
    delta = (x1-x0)
    gamma = g1-g0

    if (np.norm(delta)<tol):
        print ("value has converged, the optimal value is ")
        x1 = np.reshape(x1, N, 2)
        break

    if(METHOD_ORDER==1):
        p = (-g1 / np.norm(g1))
    elif(METHOD_ORDER==2):
        if (np.transpose(gamma)*delta > (-2*(np.transpose(g1) * p))):
            # BFGS update
            # check if h is positive definite
            H = H + (1 / (np.transpose(gamma)*delta))*(gamma*np.transpose(gamma)) - (H * delta) * np.transpose(H * delta)/(np.transpose(delta) * H * delta)

            H = modichol(H, 0.1, 20)
            p = plt.linalg.solve(-H, g1)
            p = p / np.norm(p)

    a = 1/2
    a1 = armijo_condition(funeval(), J, g1, u[:, kk], p, a)

    # % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    u[:, kk + 1] = u[:, kk] + a * p
    J0 = J
    g0 = g1
    x0 = x1
    J1[:, kk] = J

y[0,:] = np.zeros((1,5))
y[1:10,:] = x

ax1 = plt.figure()
ax1.plot(y[:,0], y[:,1],'-.')
ax1.plot(y[0,0],y[0,1],'rs')
ax1.plot(1,1,'bp')