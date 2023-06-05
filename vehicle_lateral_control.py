import casadi as ca
import casadi.tools as ca_tools
import math
import time

import numpy as np

def shift_movement(T, t0, x0, u, f):
    f_value = f(x0, u[0, :])
    state_next_ = x0 + T*f_value.T
    t_ = t0 + T
    u_next_ = ca.vertcat(u[1:, :], u[-1, :])

    return t_, state_next_, u_next_

if __name__ == '__main__':
    T = 0.2 # sampling time [s]
    N = 15 # prediction horizon
    vx_max = 30
    vx_min = 0
    vy_max = 5
    vy_min = -5
    Y_max = 1.75
    Y_min = -1.75

    ax_max = 2 * math.sqrt(2)
    ax_min = -2 * math.sqrt(2)
    df_max = np.pi/18
    df_min = -np.pi/18

    lf = 1.2
    lr = 1.6
    m = 1575
    Iz = 2875
    Cf_0 = -25000
    Cr_0 = -50000
    vx = ca.SX.sym('vx')
    vy = ca.SX.sym('vy')
    pd = ca.SX.sym('pd')
    p = ca.SX.sym('p')
    X_g = ca.SX.sym('X_g')
    Y_g = ca.SX.sym('Y_g')


    aopt_f = 20 * np.pi/180
    aopt_r = 11 * np.pi/180
    Fymax_f = Cf_0 * aopt_f/2
    Fymax_r = Cr_0 * aopt_r/2
    states = ca.vertcat(vx, vy, pd, p, X_g, Y_g)
    n_states = states.size()[0]

    ax = ca.SX.sym('ax')
    df = ca.SX.sym('df')
    controls = ca.vertcat(ax, df)
    n_controls = controls.size()[0]

    af = df - (vy + lf * pd) / vx
    ar = - (vy - lr * pd) / vx
    Cf = Fymax_f * 2 * aopt_f / (aopt_f ** 2 + af ** 2)
    Cr = Fymax_r * 2 * aopt_r / (aopt_r ** 2 + ar ** 2)

    Fcf = -Cf * af
    Fcr = -Cr * ar

    ## rhs
    rhs = ca.horzcat(pd*vy + ax)
    rhs = ca.horzcat(rhs, -pd*vx + 2/m*(Fcf * ca.cos(df)+Fcr))
    rhs = ca.horzcat(rhs, 2/Iz*(lf*Fcf-lr*Fcr))
    rhs = ca.horzcat(rhs, pd)
    rhs = ca.horzcat(rhs, vx*ca.cos(p)-vy*ca.sin(p))
    rhs = ca.horzcat(rhs, vx*ca.sin(p)+vy*ca.cos(p))

    ## function
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    U = ca.SX.sym('U', N, n_controls)

    X = ca.SX.sym('X', N+1, n_states)

    P = ca.SX.sym('P', n_states+2)

    Q = 1000
    R = np.array([[1, 0.0], [0.0, 1]])


    ### define
    X[0, :] = P[:6] # initial condiction

    #### define the relationship within the horizon
    for i in range(N):
        f_value = f(X[i, :], U[i, :])
        X[i+1, :] = X[i, :] + f_value*T

    ##ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])
    #### cost function
    obj = 0 #### cost
    for i in range(N):
        obj = obj + Q*(X[i+1, 0]-P[6])**2
        obj = obj + Q*(X[i+1, 5]-P[7])**2
        obj = obj + ca.mtimes([U[i, :], R, U[i, :].T])
        ##obj = obj + ca.mtimes(X[i+1, 0]-P[6],Q,X[i+1, 0]-P[6])
        ##obj = obj + ca.mtimes(X[i+1, 5]-P[7],Q,X[i+1, 5]-P[7])


    #### constrains
    g = [] # equal constrains
    for i in range(N+1):
        g.append(X[i, 0])
        g.append(X[i, 1])
        g.append(X[i, 5])
        if i>0 and i<N:
            g.append(U[i, 0]-U[i-1, 0])
            g.append(U[i, 1]-U[i-1, 1])


    nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    lbg = []
    ubg = []

    for j in range(N+1):
        lbg.append(vx_min)
        ubg.append(vx_max)
        lbg.append(vy_min)
        ubg.append(vy_max)
        lbg.append(Y_min)
        ubg.append(Y_max)
        if j>0 and j<N:
            lbg.append(-3*T)
            ubg.append(1.5*T)
            lbg.append(-np.pi/36*T)
            ubg.append(np.pi/36*T)

    lbx = []
    ubx = []
    for _ in range(N):
        lbx.append(ax_min)
        ubx.append(ax_max)
    for _ in range(N):
        lbx.append(df_min)
        ubx.append(df_max)


    # Simulation
    t0 = 0.0
    x0 = np.array([20, 0, 0, 0, 0, -1.75]).reshape(-1, 1)# initial state
    xs = np.array([30, 1.75]).reshape(-1, 1) # final state
    u0 = np.array([1,2]*N).reshape(-1, 2)# np.ones((N, 2)) # controls
    xh = [x0] # contains for the history of the state
    uh = []
    th = [t0] # for the time
    sim_time = 15
    T_sim=0.1
    ## start MPC
    mpciter = 0
    start_time = time.time()
    while(mpciter-sim_time/T_sim<0.0 ):
        ## set parameter
        if mpciter-sim_time/(2*T_sim)>=0.0:
            xs = np.array([20, -1.75]).reshape(-1, 1)
        c_p = np.concatenate((x0, xs))
        init_control = ca.reshape(u0, -1, 1)
        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
        u_sol = ca.reshape(res['x'],  N, n_controls)
        u_attach=ca.reshape(u_sol[0, :], -1, 1)
        uh.append(u_attach.full())
        th.append(t0)
        t0, x0, u0 = shift_movement(T_sim, t0, x0, u_sol, f)
        x0 = ca.reshape(x0, -1, 1)
        xh.append(x0.full())
        mpciter = mpciter + 1
    print(time.time() - start_time)


    import matplotlib.pyplot as plt

    # Extract vx values and time from xh and th
    vx_values = [state[0] for state in xh]
    time_values = th

    # Extract Y_g values from xh
    Y_g_values = [state[5] for state in xh]

    # Extract ax and df values from uh
    ax_values = [control[0] for control in uh]
    df_values = [control[1] for control in uh]


    # Create a 2x2 grid of subplots
    fig, ax = plt.subplots(2, 2, figsize=(10, 10))

    # Plot vx vs time
    ax[0, 0].plot(time_values, vx_values)
    ax[0, 0].set_xlabel('T')
    ax[0, 0].set_ylabel(r'$v_{x}$')
    ax[0, 0].grid(True)

    # Plot Y_g vs time
    ax[0, 1].plot(time_values, Y_g_values)
    ax[0, 1].set_xlabel('T(s)')
    ax[0, 1].set_ylabel(r'$Y$')
    ax[0, 1].grid(True)

    # Plot ax vs time
    ax[1, 0].plot(time_values[:-1], ax_values)
    ax[1, 0].set_xlabel('T(s)')
    ax[1, 0].set_ylabel(r'$a_{x}$')
    ax[1, 0].grid(True)

    # Plot df vs time
    ax[1, 1].plot(time_values[:-1], df_values)
    ax[1, 1].set_xlabel('T(s)')
    ax[1, 1].set_ylabel(r'$\delta_{f}$')
    ax[1, 1].grid(True)

    # Adjust layout and show the plot
    plt.tight_layout()
    plt.show()