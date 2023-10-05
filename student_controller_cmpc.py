import numpy as np
import cvxpy as cp

def Setup_Derivative(x_bar, u_bar, param, control=False):
    ## this function is optional
    h = param["h"]
    T = param["T"]
    L_f = param["L_f"]
    L_r = param["L_r"]
    a_lim = param["a_lim"]
    delta_lim = param["delta_lim"]

    dt = h
    x = x_bar[0]
    y = x_bar[1]
    psi = x_bar[2]
    v = x_bar[3]
    delta = u_bar[0]
    alpha = u_bar[1]
    beta = np.arctan((L_r * np.arctan(delta)) / (L_f + L_r))

    A = np.zeros([4, 4])
    B = np.zeros([4, 2])
    result = []

    if control==False:
        A = np.array([
            [0, 1, -dt * v * np.sin(psi + beta), dt * v * np.cos(psi + beta)],
            [0, 1, dt * v * np.cos(psi + beta), dt * v * np.sin(psi + beta)],
            [0, 0, 1, (dt * np.arctan(delta)) / (((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1)**(1/2) * (L_f + L_r))],
            [0, 0, 0, 1]
        ])

        result = A
    else:
        B = np.array([
            [0, -(dt * L_r * v * np.sin(psi + beta)) / ((delta**2 + 1) * ((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1) * (L_f + L_r))],
            [0, (dt * L_r * v * np.cos(psi + beta)) / ((delta**2 + 1) * ((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1) * (L_f + L_r))],
            [0, (dt * v) / ((delta**2 + 1) * ((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1)**(3/2) * (L_f + L_r))],
            [dt, 0]
        ])
        result = B

    ## TODO

    return result # TODO

def Student_Controller_LQR(x_bar, u_bar, x0, Fun_Jac_dt, param):
    dim_ctrl = u_bar.shape[1]
    dim_state = x_bar.shape[1]
    N = len(x_bar) - 1  # Preview horizon

    p = 5
    q = 300
    r = 20

    P = np.eye(dim_state) * p
    Q = np.eye(dim_state) * q
    R = np.eye(dim_ctrl) * r

    # Initialize variables
    delta_s = []  # State perturbations
    delta_u = []  # Control perturbations

    for k in range(N):
        delta_s.append(cp.Variable(4))
        delta_u.append(cp.Variable(2))
    delta_s.append(cp.Variable(4))

    # Define the cost function
    cost = 0
    for k in range(N):
        cost += cp.quad_form(delta_s[k], Q) + cp.quad_form(delta_u[k], R)
    cost += cp.quad_form(delta_s[N], P)  # Terminal cost

    # Define the constraints
    constraints = []
    for k in range(N):
        A_k = Fun_Jac_dt(x_bar[k], u_bar[k], control=False)  # Compute A_k matrix
        B_k = Fun_Jac_dt(x_bar[k], u_bar[k], control=True)  # Compute B_k matrix

        # State dynamics constraint
        state_constraint = delta_s[k + 1] == A_k @ delta_s[k] + B_k @ delta_u[k]
        constraints.append(state_constraint)

    # Initial state constraint
    constraints.append(delta_s[0] == x0 - x_bar[0])

    # Create the optimization problem
    prob = cp.Problem(cp.Minimize(cost), constraints)

    # Solve the problem
    prob.solve()

    delta_u_opt = [u_var.value for u_var in delta_u]
    u_act = u_bar[0] + np.array(delta_u_opt[0])

    return u_act

    return u_act # TODO 

def Student_Controller_CMPC(x_bar, u_bar, x0, Fun_Jac_dt, param):
    # TODO

    return # TODO

    # A = np.zeros([4, 4])
    # B = np.zeros([4, 2])

    # dt = h
    # x = x0[0]
    # y = x0[1]
    # psi = x0[2]
    # v = x0[3]
    # delta = u0[0]
    # alpha = u0[1]

    # A[:, 0] = 0
    # A[0:2, 1] = 1
    # A[2:4, 1] = 0
    # A[2, 2] = 1
    # A[3, 2] = 0
    # A[3, 3] = 1
    # A[0, 2] = -dt * v * np.sin(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))
    # A[0, 3] = dt * np.cos(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))
    # A[1, 2] = dt * v * np.cos(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))
    # A[1, 3] = dt * np.sin(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))
    # A[2, 3] = (dt * np.arctan(delta)) / (((L_r**2 * (np.arctan(delta))**2) / (L_f+L_r)**2 + 1)**(1/2) * (L_f+L_r))

    # B[0:3, 0] = 0
    # B[3, 0] = dt
    # B[3, 1] = 0
    # B[0, 1] = -(dt*L_r*v*np.sin(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))) / ((delta**2+1)*((L_r**2*(np.arctan(delta))**2) / (L_f+L_r)**2 + 1) * (L_f+L_r))
    # B[1, 1] = (dt*L_r*v*np.cos(psi + np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))) / ((delta**2+1)*((L_r**2*(np.arctan(delta))**2) / (L_f+L_r)**2 + 1) * (L_f+L_r))