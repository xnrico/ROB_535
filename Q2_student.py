def sim_car():
    import numpy as np
    from scipy.integrate import solve_ivp
    # Constants
    L = 2  # Length from front to rear axle

    ### student code starts here
    # System dynamics
    def car_model(t, y, u):
        p1, p2, psi, v = y
        delta, a = u
        dp1_dt = v * np.cos(psi) # TODO
        dp2_dt = v * np.sin(psi) # TODO
        dpsi_dt = v / L * np.tan(delta) # TODO
        dv_dt = a # TODO
        return [dp1_dt, dp2_dt, dpsi_dt, dv_dt]
    
    # Initial state
    y0 = [0, 0, 0, 1]   # [x, y, psi, v]
    # Control inputs
    u = [0.1, 0.1]     # [delta, a]

    # Integrate system dynamics
    t_span = [0, 10] # 10 seconds
    # Time points for evaluation (t_eval)
    t_eval = np.arange(0, 10, 0.01)
    sol = solve_ivp(lambda t, y: car_model(t, y, u), t_span, y0, t_eval=t_eval)

    return sol

def sim_car_control():
    import numpy as np
    from scipy.integrate import solve_ivp
    # Constants
    L = 2  # Length from front to rear axle

    # System dynamics
    def car_model(t, y, delta_func, a_func):
        p1, p2, psi, v = y
        delta = delta_func # TODO
        a = a_func # TODO
        dp1_dt = v * np.cos(psi) # TODO
        dp2_dt = v * np.sin(psi) # TODO
        dpsi_dt = v / L * np.tan(delta) # TODO
        dv_dt = a # TODO
        return [dp1_dt, dp2_dt, dpsi_dt, dv_dt]

    # Initial state
    y0 = [0, 0, 0, 1]   # [x, y, psi, v]

    # Periodic control inputs
    def periodic_delta(t):
        t = np.array(t)
        return 0.1 * np.sin(0.5 * t)

    def periodic_a(t):
        t = np.array(t)
        return 0.1 * np.cos(0.25 * t)

    # Integrate system dynamics
    t_span = [0, 20]  # 20 seconds
    # Time points for evaluation (t_eval)
    t_eval = np.arange(0, 20, 0.01)
    sol = solve_ivp(lambda t, y: car_model(t, y, periodic_delta(t), periodic_a(t)), t_span, y0, t_eval=t_eval)

    return sol

def sim_car_euler():
    import numpy as np
    # Constants
    L = 2  # Length from front to rear axle
    dt = 0.01  # Time step

    # System dynamics
    def car_model(y, delta, a):
        p1, p2, psi, v = y
        dp1_dt = v * np.cos(psi) # TODO
        dp2_dt = v * np.sin(psi) # TODO
        dpsi_dt = v / L * np.tan(delta) # TODO
        dv_dt = a # TODO
        return [dp1_dt, dp2_dt, dpsi_dt, dv_dt]

    # Periodic control inputs
    def periodic_delta(t):
        t = np.array(t)
        return 0.1 * np.sin(0.5 * t)

    def periodic_a(t):
        t = np.array(t)
        return 0.1 * np.cos(0.25 * t)

    # Initial state
    y = np.array([0, 0, 0, 1])  # [x, y, psi, v]

    # Simulation
    t_max = 20
    num_steps = int(t_max / dt)
    results = np.zeros((num_steps, 4))
    time = np.zeros(num_steps)

    for i in range(num_steps):
        delta = periodic_delta(i * dt)
        a = periodic_a(i * dt)
        results[i] = y
        time[i] = i * dt
        # predict the next state using explicit Euler integration scheme
        y = y + dt * car_model(y, delta, a) # TODO

    return time, results