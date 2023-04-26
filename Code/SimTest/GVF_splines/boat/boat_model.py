import numpy as np

# Modelo boat
def boat_model(t,x,f,tau,chi,mumat,mutau,waterspeed): # tau = u_ctrl
    
    # Drag matrix
    mu = mumat
    
    # E
    E = np.array([[0,-1],[1,0]])
    # torque coeff
    mut = mutau
    
    #speed
    vx     = x[0]
    vy     = x[1]
    v      = np.array([vx,vy])
    nv     = np.sqrt(vx**2+vy**2)
    #pos
    rx     = x[2]
    ry     = x[3]
    #r      = np.array([rx,ry])
    #ang speed
    omega  = x[4]
    
    # Elements of Rot Matrix (orientation)
    R00 = x[5]
    R01 = x[6]
    R10 = x[7]
    R11 = x[8]
    R = np.array([[R00,R01],[R10,R11]])
    
    # augmented dynamics
    w   = x[9]
    
    # unitary vector
    ux = np.array([1,0])

    
    xdot = np.zeros(x.shape)
    
    # Boat model
    dv = (R @ ux)*f - (R @ mu @ R.T @ (v-waterspeed))
    dr = v
    domega = tau - mut*omega
    dR = omega*(E @ R)
    
    xdot[0] = dv[0]; xdot[1] = dv[1]; xdot[2] = dr[0]; xdot[3] = dr[1]
    xdot[4] = domega;
    xdot[5] = dR[0,0]; xdot[6] = dR[0,1]; xdot[7] = dR[1,0]; xdot[8] = dR[1,1]
    
    xdot[9] = chi[2]/np.sqrt(chi[0]**2 + chi[1]**2) # 3d dim dynamics
    
    return xdot
    