import numpy as np

# Modelo UAV
def UAV_model(t,x,airspeed,u_ctrl,chi):
    
    px  = x[0]
    py  = x[1]
    psi = x[2]
    w   = x[3]
    
    xdot = np.zeros(x.shape)
    
    xdot[0] = airspeed*np.cos(psi)# + 0.2 # wind
    xdot[1] = airspeed*np.sin(psi)# - 0.4 # wind
    xdot[2] = u_ctrl;
    xdot[3] = airspeed*chi[2]/np.sqrt(chi[0]**2 + chi[1]**2) # 3d dim dynamics
    
    return xdot