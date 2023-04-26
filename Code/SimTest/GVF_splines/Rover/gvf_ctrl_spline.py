


import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Factor "escalado" para w
L = 0.1;

# Velocity Vector Field
def vector_field(p, dp, s, psi, k1, k2, kyaw, sx, sy, w):
    
    px = p[0]; py = p[1];
    
    sxw = sx(w); syw = sy(w);
    dsx  = sx.derivative(1)(w); dsy = sy.derivative(1)(w);
    
    phi1 = px - sxw; phi2 = py - syw;
    
    # Guidance vector
    chi = np.zeros(3); chip = np.zeros(2)
    chi[0] = L*(dsx*L**2 - k1*phi1)
    chi[1] = L*(dsy*L**2 - k2*phi2)
    chi[2] = L*(L**2 + k1*phi1*dsx + k2*phi2*dsy)
    
    chip[0] = chi[0]; chip[1] = chi[1];
    
    return chi,chip, phi1, phi2

# Control yaw rate signal (s = airspeed...), psi in rads
def u_ctrl(p, dp, s, psi, k1, k2, kyaw, sx, sy, w):
    
    E      = np.array([[0, -1], [1,0]])
     
    px = p[0]; py = p[1]; dpx = dp[0]; dpy = dp[1]
    
    sxw = sx(w); syw = sy(w);
    dsx  = sx.derivative(1)(w); dsy = sy.derivative(1)(w);
    ddsx = sx.derivative(2)(w); ddsy = sy.derivative(2)(w);
    
    # Path
    gradphi1 = np.zeros(3); gradphi2 = np.zeros(3);
    
    phi1 = px - sxw; phi2 = py - syw;
    
    gradphi1[0] = 1; gradphi1[1] = 0; gradphi1[2] = dsx;
    gradphi2[0] = 0; gradphi2[1] = 1; gradphi2[2] = dsy;
    
    # Guidance vector
    
    chi = np.zeros(3); chip = np.zeros(2)
    chi[0] = L*(dsx*L**2 - k1*phi1)
    chi[1] = L*(dsy*L**2 - k2*phi2)
    chi[2] = L*(L**2 + k1*phi1*dsx + k2*phi2*dsy)
    chip[0] = chi[0]; chip[1] = chi[1];
    
    # Jacobian
    J = np.zeros((2,3))
    J[0,0] = -k1*L; J[0,1] = 0;  J[0,2] = (ddsx*L**2 + k1*dsx*L)
    J[1,0] = 0;  J[1,1] = -k2*L; J[1,2] = (ddsy*L**2+ k2*dsy*L)
    
    
    # Control action
    dw = s*chi[2]/np.sqrt(chi[0]**2 + chi[1]**2) # 3d dim dynamics
    dzeta = np.array([dpx, dpy, dw])
    h = np.array([np.cos(psi), np.sin(psi)])
    
    modchip = np.sqrt(chip[0]**2 + chip[1]**2)
    hatchip = chip/modchip
    
    u2 =  -kyaw* (h.T @ E @ hatchip) # A @ B = np.
    u1 =  (hatchip.T @ E @ J @ dzeta)/modchip
    
    return -u1+u2
    
