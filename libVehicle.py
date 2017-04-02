import numpy as np

def vehicle_time_march(x, u, l):
    # actual system as function of time
    # see Lumun

    dx = np.zeros(5)
    dx[0] = (x[3]+x[4])*np.cos(x[2])*0.5
    dx[1] = (x[3]+x[4])*np.sin(x[2])*0.5
    dx[2] = (1/l)*(x[3]-x[4])
    dx[3] = u[0]
    dx[4] = u[1]
    return (dx)

def armijo_condition(fun, f1, g1, u, p, a):
    gamma = 0.9
    eta = 0.5

    # armijo condition
    [f2] = fun(u+a*p)
    while (f2>f1+eta*a*np.transpose(g1)*p):
        a = gamma*a
        if (np.norm(a*p)<1e-4):
            break

        f2 = fun(u+a*p)
    return (a)

def firstdiff(qx, qy, pp):
    phi = np.empty(0)
    dphi = np.empty(0)
    n = qx.shape[0]

    for kk in range(0, n):
        phi = np.sum(phi) + np.sum(function_eval(qx[kk], qy[kk])) # j is the whole function

    if (pp!=0):
        for kk in range(0, n):
            dphi = np.append(dphi, gradient_eval(qx(kk),qy(kk)))

    if (pp==1):
        dphi = np.append(dphi, [0, 0, 0])

    return(phi, dphi)

def adjoint(lmd, x, l):
    # linearised model of dynamics
    A = [[0, 0, -(x[3]+x[4])*np.sin(x[2])*0.5, 0.5*np.cos(x[2]), 0.5*np.cos(x[2])],
    [0, 0, (x[3]+x[4])*np.cos(x[2])*0.5, 0.5*np.sin(x[2]), 0.5*np.sin(x[2])],
    [0, 0, 0, 1.0/l, -1.0/l],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]]

    # get objective function and first differential
    qx = x[0]
    qy = x[1]
    [p, dj] = firstdiff(qx, qy, 1)

    lambda1 = 1.0
    for ii in range(0, 15):
        lambda1 = lambda1/2.0
        value =  dj - np.transpose(lmd*(A+lambda1*np.eye((A.shape[0], A.shape[1])))) #check the signs, no - before dj previously
    return(value)

def RK4_lambda(lmd, adj, h):
    f1 = adj(lmd)
    f2 = adj(lmd+h/2.0*np.transpose(f1))
    f3 = adj(lmd+h/2.0*np.transpose(f2))
    f4 = adj(lmd+h*np.transpose(f3))
    lmd1 = lmd+h/6.0*(np.transpose(f1)+2.0*np.transpose(f2)+2.0*np.transpose(f3)+np.transpose(f4))
    return(lmd1)


def gfun(x, N, h):
    lmd = np.zeros((N + 1, 5))  # why are lmd zeros? shouldn't it be the deviation from destination?

    lmd[N + 1, :] = [0, 0, 0, 0, 0]

    for i in range(N, 2, -1):
        adj = adjoint(lmd, x[i, :])
        lmd[i - 1, :] = RK4_lambda(lmd[i, :], adj, -h)

        B = [[0, 0, 0, 1, 0], [0, 0, 0, 0, 1]]
        gradient = -lmd * B  # is something missing here? check the sign, there was a -
        gradient[-1, :] = []
        gradient = np.reshape(gradient, 2 * N, 1)
        x[-1, :] = []
    return (gradient, x)

def funeval(u, h, N):
    N = u.shape[0]/2
    u = np.reshape(u, N, 2)
    x = np.zeros((N,5))
    [J,dj] = firstdiff(x[:,0], x[:,1], 2)


    for ii in range(0, N):
        fun_dynamic = vehicle_time_march(x, u[ii,:]) # vehicle time march is obtained from Laumond
        x[ii+1,:] = RK4_timemarch(x[ii,:], fun_dynamic, h)

        J=J+firstdiff(x(ii,1),x(ii,2),0)
    return(J, x, dj)

def armijo_condition(funeval, J, g1, u, p, a):
    return(J)

def RK4_timemarch(qx, qy, h):
    return ([qx, qy, h])

def function_eval(qx, qy):
    return([qx, qy])

def gradient_eval(qx, qy):
    return([qx, qy])