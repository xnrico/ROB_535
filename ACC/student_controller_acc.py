def Student_Controller(t, x, param):
    import numpy as np
    vd = param["vd"]
    v0 = param["v0"]

    m = param["m"]
    Cag = param["Cag"]
    Cdg = param["Cdg"]
    
    D = x[0]
    v = x[1]

    ## TODO
    lam = 0.8
    alpha = 3
    w = 100
    h = (v - vd)**2 / 2
    B = D - 1/2 * (v0 - v)**2 / Cdg - 1.8*v
    # other ...

    ## TODO
    P = np.array([[2, 0], [0, 2*w]])
    
    ## TODO
    A = np.array([[(v - vd)/m, -1],
                  [1/m * (1.8 + (v-v0)/Cdg), 0],
                  [1/m , 0],
                  [-1/m , 0],
                  [0, -1]
                  ])

    ## TODO
    b = np.array([-lam*h, alpha*B + (v0-v), Cag, Cdg, 0])

    ## TODO
    q = np.zeros([2, 1])
    
    return A, b, P, q