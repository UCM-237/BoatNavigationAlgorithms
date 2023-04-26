import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Factor "escalado" para w
L = 1;


# Velocity Vector Field
def vector_field(p, k1, k2, sx, sy, w):
    
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
def u_ctrl(p, dp, ddp, R00,R01,R10,R11, omega, mutau, k1, k2, ktau, sx, sy, w):
    

    E      = np.array([[0, -1], [1,0]])
    R      = np.array([[R00,R01],[R10,R11]])
    ux     = np.array([1,0])
    
    px = p[0]; py = p[1]; dpx = dp[0]; dpy = dp[1]
    nv = np.sqrt(dpx**2+dpy**2)
    ddpx = ddp[0]; ddpy = ddp[1]
    ndv = np.sqrt(ddpx**2+ddpy**2)
    
    sxw = sx(w); syw = sy(w);
    dsx  = sx.derivative(1)(w); dsy = sy.derivative(1)(w);
    ddsx = sx.derivative(2)(w); ddsy = sy.derivative(2)(w);
    
    # Path
    gradphi1 = np.zeros(3); gradphi2 = np.zeros(3);
    
    phi1 = px - sxw; phi2 = py - syw;
    
    gradphi1[0] = 1; gradphi1[1] = 0; gradphi1[2] = dsx;
    gradphi2[0] = 0; gradphi2[1] = 1; gradphi2[2] = dsy;
    
    # Guidance vector and its time derivative
    
    chi = np.zeros(3); chip = np.zeros(2); 
    dchi = np.zeros(3); dchip = np.zeros(2)
        # Gudiance vector field
    chi[0] = L*(dsx*L**2 - k1*phi1)
    chi[1] = L*(dsy*L**2 - k2*phi2)
    chi[2] = L*(L**2 + k1*phi1*dsx + k2*phi2*dsy)
    chip[0] = chi[0]; chip[1] = chi[1];
        # Vector field time derivative
    dchi[0] = -L*k1*dpx; dchi[1] = -L*k2*dpy;
    dchi[2] = L*(k1*dsx*dpx + k2*dsy*dpy)
    dchip[0] = dchi[0]; dchip[1] = dchi[1]
    modchip = np.sqrt(chip[0]**2 + chip[1]**2)
    hatchip = chip/modchip
    
    # Jacobian
    J = np.zeros((2,3))
    J[0,0] = -k1*L; J[0,1] = 0;  J[0,2] = (ddsx*L**3 + k1*dsx*L)
    J[1,0] = 0;  J[1,1] = -k2*L; J[1,2] = (ddsy*L**3+ k2*dsy*L)
    
    
    # aux dynamics
    dw = chi[2]/np.sqrt(chi[0]**2 + chi[1]**2) # 3d dim dynamics
    ddw = (dchi[2] + chi[2]*(chi[0]*dchi[0] + chi[1]*dchi[1]))/modchip
          
    dzeta = np.array([dpx, dpy, dw])
    ddzeta  = np.array([ddpx,ddpy,ddw])

    # desired angular speed
    dtheta = -(hatchip.T @ E @ J @ dzeta)/modchip
    
    # auxiliary variables
    dhatchip = -dtheta*(E @ chip)
    dinvmodchip = (chip.T @ dchip)/modchip
    
    a = (dhatchip.T/modchip + dinvmodchip*hatchip.T) @ E @ J @ dzeta
    b = (hatchip.T @ E @ J @ ddzeta)/modchip
    ddtheta = -(a+b)
    
    V = 0.5*((omega - dtheta)**2 + (R @ ux - hatchip).T @ (R @ ux - hatchip))
    
    domg = ddtheta + hatchip.T @ E @ R @ ux - ktau*(omega-dtheta)#*(hatchip.T @ E @ R @ ux)**2
    
    orierr = (R @ ux - hatchip).T @ (R @ ux - hatchip)
    tau = mutau*omega + domg
    
    
    return V,(orierr),dtheta,tau
    
