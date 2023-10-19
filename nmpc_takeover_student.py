import casadi as ca
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
import time

def nmpc_controller():
    # Declare simulation constants
    T = 3 # TODO: You are supposed to design the planning horizon
    N = 40 # TODO  You are supposed to design the planning horizon
    h = T / N # TODO: What is the time interval for simulation? 

    # system dimensions
    Dim_state = 4 # TODO
    Dim_ctrl  = 2 # TODO

    # additional parameters
    x_init = ca.MX.sym('x_init', (Dim_state, 1)) # initial condition, # the state should be position to the leader car
    v_leader = ca.MX.sym('v_leader', (2, 1))      # leader car's velocity w.r.t ego car
    v_des = ca.MX.sym('v_des')
    delta_last = ca.MX.sym('delta_last')
    par = ca.vertcat(x_init, v_leader, v_des, delta_last)
    
    # Continuous dynamics model
    x_model = ca.MX.sym('xm', (Dim_state, 1))
    u_model = ca.MX.sym('um', (Dim_ctrl, 1))

    L_f = 1.0 # Car parameters, do not change
    L_r = 1.0 # Car parameters, do not change

    beta = ca.atan(L_r/(L_f+L_r)*ca.atan(u_model[1])) # TODO 

    xdot = ca.vertcat(
            x_model[3] * ca.cos(x_model[2]+beta) - v_leader[0],
            x_model[3] * ca.sin(x_model[2]+beta),
            x_model[3]/L_r * ca.sin(beta),
            u_model[0]
            ) # TODO

    # Discrete time dynmamics model
    Fun_dynmaics_dt = ca.Function('f_dt', [x_model, u_model, par], [xdot*h + x_model]) # TODO
    
    # Declare model variables, note the dimension
    x = ca.MX.sym('x', (Dim_state, N + 1)) # TODO
    u = ca.MX.sym('u', (Dim_ctrl , N)) # TODO

    # Keep in the same lane and take over it while maintaing a high speed
    p = 1e5
    q = 1e3
    r = 1

    P = p * ((v_des - x_model[3])**2 + x_model[1]**2) # TODO
    L = q * (v_des - x_model[3])**2 + q * xdot[1]**2 + q*0.01 * xdot[2]**2 + q*5 * x_model[1]**2 + r * u_model[0]**2 + 0.5*r * u_model[1] # TODO

    Fun_cost_terminal = ca.Function('P', [x_model, par], [P])
    Fun_cost_running = ca.Function('Q', [x_model, u_model, par], [L])

    # state and control constraints
    state_ub = np.array([ 1e4,  3,  3.14,  200]) # TODO 
    state_lb = np.array([ -1e4,  -1,  -3.14,  -200]) # TODO  ???
    ctrl_ub  = np.array([4, 0.6]) # TODO 
    ctrl_lb  = np.array([-10, -0.6]) # TODO 
    
    # upper bound and lower bound
    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)

    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)

    ub_var = np.concatenate((ub_u.reshape((Dim_ctrl * N, 1)), ub_x.reshape((Dim_state * (N+1), 1)))) # TODO
    lb_var = np.concatenate((lb_u.reshape((Dim_ctrl * N, 1)), lb_x.reshape((Dim_state * (N+1), 1)))) # TODO

    # dynamics constraints: x[k+1] = x[k] + f(x[k], u[k]) * dt
    cons_dynamics = []
    ub_dynamics = np.zeros((N * Dim_state, 1)) # TODO
    lb_dynamics = np.zeros((N * Dim_state, 1)) # TODO
    for k in range(N):
        Fx = Fun_dynmaics_dt(x[:, k], u[:, k], par)
        # TODO
        for j in range(Dim_state):
            cons_dynamics.append(x[j, k+1] -  Fx[j])


    # state constraints: G(x) <= 0
    cons_state = []
    for k in range(N):
        #### collision avoidance:
        # TODO
        dist = (x[0, k]/32)**2 + (x[1, k]/2.125)**2 - 1
        cons_state.append(-dist) # TODO)

        #### Maximum lateral acceleration ####
        dx = (x[:, k+1] - x[:, k]) / h
        ay = x[3, k] * dx[2] # TODO: Compute the lateral acc using the hints
        
        gmu = (0.5 * 0.6 * 9.81)
        cons_state.append(ay - gmu) # TODO)
        cons_state.append(-ay - gmu) # TODO)

        #### lane keeping ####
        cons_state.append(x[1, k] - 2.75) # TODO)
        cons_state.append(-x[1, k] - 1) # TODO)

        #### steering rate ####
        if k >= 1:
            d_delta = (u[1, k] - u[1, k-1]) / h # TODO
            cons_state.append(d_delta - 0.55)  # TODO)
            cons_state.append(-d_delta - 0.55)  # TODO)
        else:
            d_delta = u[1, 0] - delta_last # TODO, for the first input, given d_last from param
            cons_state.append(d_delta - 0.55)  # TODO)
            cons_state.append(-d_delta - 0.55)  # TODO)

    ub_state_cons = np.zeros((len(cons_state), 1))
    lb_state_cons = np.zeros((len(cons_state), 1)) - 1e5

    # cost function: # NOTE: You can also hard code everything here
    J = Fun_cost_terminal(x[:, -1], par) # + p * ((x[3, -1]*np.sin(x[2, -1]+(np.arctan(L_r/(L_f+L_r)*np.arctan(u[1,-1])))))**2 + (x[3,-1]/L_r*np.sin(np.arctan(L_r/(L_f+L_r)*np.arctan(u[1,-1]))))**2)
    for k in range(N):
        J = J + Fun_cost_running(x[:, k], u[:, k], par)

    # initial condition as parameters
    cons_init = [x[:, 0] - x_init]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))
    
    # Define variables for NLP solver
    vars_NLP   = ca.vertcat(u.reshape((Dim_ctrl * N, 1)), x.reshape((Dim_state * (N+1), 1)))
    cons_NLP = cons_dynamics + cons_state + cons_init
    cons_NLP = ca.vertcat(*cons_NLP)
    lb_cons = np.concatenate((lb_dynamics, lb_state_cons, lb_init_cons))
    ub_cons = np.concatenate((ub_dynamics, ub_state_cons, ub_init_cons))

    # Create an NLP solver
    prob = {"x": vars_NLP, "p":par, "f": J, "g":cons_NLP}
    
    return prob, N, vars_NLP.shape[0], cons_NLP.shape[0], par.shape[0], lb_var, ub_var, lb_cons, ub_cons  