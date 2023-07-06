import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Vector field Scale factor
kL = 1;


"""
    Velocity vector field function
    @ param:
        @ p   : boat position vector
        @ mumat : Drag matrix
        @ kx,ky : Vector fiekLd agressiveness in x and y direction constants
        @ sx,sy : spkLines functions (scipy)
        @ w     : adittionakL coordinate w
    @ returns:
        @ chi : Desired vector field
        @ chip : Physical vector field chip = [chi[0],chi[1]]
        @ phix,phiy : Distance error to splines -> Desired Path functions
"""        
def vector_field(p, kx, ky, sx, sy, w):
    
    px = p[0]; py = p[1];
    
    sxw = sx(w); syw = sy(w);
    dsx  = sx.derivative(1)(w); dsy = sy.derivative(1)(w);
    
    phix = px - sxw; phiy = py - syw;
    
    # Guidance vector
    chi = np.zeros(3); chip = np.zeros(2)
    chi[0] = kL*(dsx*kL**2 - kx*phix)
    chi[1] = kL*(dsy*kL**2 - ky*phiy)
    chi[2] = kL*(kL**2 + kx*phix*dsx + ky*phiy*dsy)
    
    chip[0] = chi[0]; chip[1] = chi[1];
    
    return chi,chip, phix, phiy

 
"""
    GVF heading control and speed controller
    @ param:
        @ p   : boat position vector
        @ dp  : boat velocity 
        @ ddp : boat accekLeration vector
        @ Rij : Rotation matrix entries
        @ omega : Boat angukLar speed
        @ mutau : angukLar drag coefficient
        @ mumat : Drag matrix
        @ kx,ky : Vector field agressiveness in x and y direction constants
        @ ktau  : Torque gain constant (torque control action)
        @ sx,sy : spkLines functions (scipy)
        @ w     : adittionakL coordinate w
    @ returns:
        @ V : Lyapunov function
        @ orierr : (hat_v - \chi^p).T x (hat_v - \chi^p)
        @ dtheta : vector fiekLd rotation rate
        @ tau    : Angular control signal
"""
def gvf_speed_control(p, dp, ddp, R00,R01,R10,R11, omega, mutau,
                      mumat, kx, ky, ktau, kf, sx, sy, w,z2,z3,z4):
    
    p = np.array(p); dp = np.array(dp); ddp = np.array(ddp)

    E      = np.array([[0, -1], [1,0]])
    R      = np.array([[R00,R01],[R10,R11]])
    ux     = np.array([1,0])
    
    # px = p[0]; py = p[1]; 
    dpx = dp[0]; dpy = dp[1]
    nv = np.sqrt(dpx**2+dpy**2)
    ddpx = ddp[0]; ddpy = ddp[1]
    ndv = np.sqrt(ddpx**2+ddpy**2)
    
    #sxw = sx(w); syw = sy(w);
    dsx   = sx.derivative(1)(w); dsy = sy.derivative(1)(w);
    ddsx  = sx.derivative(2)(w); ddsy = sy.derivative(2)(w);
    dddsx = sx.derivative(3)(w); dddsy = sy.derivative(3)(w);
    
    # curvatura
    kappa = (dsx*ddsy - dsy*ddsx)/(dsx**2 + dsy**2)**(1.5)
    dotkappa = (dsx*dddsy - dsy*dddsx)/(dsx**2 + dsy**2)**(1.5)- 3*(dsx*ddsy - dsy*ddsx)*(dsx*ddsx + dsy*ddsy)/((dsx**2 + dsy**2)**(2.5))
    
    # Get vector field and desired path
    chi, chip, phix, phiy = vector_field(p, kx, ky, sx, sy, w)
    modchip = np.sqrt(chip[0]**2 + chip[1]**2)
    hatchip = chip/modchip

    # Vector field time derivative
    dchi = np.zeros(3); dchip = np.zeros(2)
    dchi[0] = -kL*kx*dpx; dchi[1] = -kL*ky*dpy;
    dchi[2] = kL*(kx*dsx*dpx + ky*dsy*dpy)
    dchip[0] = dchi[0]; dchip[1] = dchi[1]

    # Jacobian
    J = np.zeros((2,3))
    J[0,0] = -kx*kL; J[0,1] = 0;  J[0,2] = (ddsx*kL**3 + kx*dsx*kL)
    J[1,0] = 0;  J[1,1] = -ky*kL; J[1,2] = (ddsy*kL**3+ ky*dsy*kL)
    
    
    # aux dynamics
    dw = chi[2]/np.sqrt(chi[0]**2 + chi[1]**2) # 3d dim dynamics
    ddw = (dchi[2] - chi[2]*(chi[0]*dchi[0] + chi[1]*dchi[1])/modchip**2)/modchip
          
    dzeta = np.array([dpx, dpy, dw])
    ddzeta  = np.array([ddpx,ddpy,ddw])

    # desired angukLar speed
    dtheta = -(hatchip.T @ E @ J @ dzeta)/modchip
    
    # auxiliary variables
    dhatchip = dtheta*(E @ chip)
    dinvmodchip = (chip.T @ dchip)/modchip**3
    
    a = (dhatchip.T/modchip - dinvmodchip*hatchip.T) @ E @ J @ dzeta
    b = (hatchip.T @ E @ J @ ddzeta)/modchip
    ddtheta = -(a+b)
    
    ###### Guidance controller ########
    # Old lyapunov function (depends on orientation)
    V = 0.5*((omega - dtheta)**2 + (R @ ux - hatchip).T @ (R @ ux - hatchip))
    
    dotomega = ddtheta + hatchip.T @ E @ R @ ux - ktau*(omega-dtheta)
    
    orierr = (R @ ux - hatchip).T @ (R @ ux - hatchip)
    tau = mutau*omega + dotomega
    
    
    ###### Velocity controlle ######
    # Spline speed controller
    #dotV_d = np.sqrt(ddsx**2+  ddsy**2);
    #V_d    = np.sqrt(dsx**2 + dsy**2);
    
    # Constant speed controller
    #dotV_d  = 0;
    #V_d     = 1.32; 
    
    # Adaptative speed controller (V_d = v_min + v_aux*exp(-eta))
    # V_d = desired speed
    v_max = 0.8; v_min = 0.8; v_aux = v_max - v_min;
    v_aux = 2.2;
    #z2 = 1.418; z3 = 2.895; z4 = 21.792; # Constants
    #z2 = 1.095; z3 = 0.119; z4 = 16.51;
    z2 = 0.0195; z3 = 2.019; z4 = 10.51;
    eta = z2*(1- dp.T/nv @ hatchip) + z3*(phix**2 + phiy**2) + z4*kappa**2
    
    doteta = -z2*(ddp.T/ndv @ hatchip + dtheta*(dp.T/nv @ E @ hatchip));
    doteta +=  2*z3*(phix*dpx + phiy*dpy)
    doteta += 2*z4*kappa*dotkappa;
    dotV_d = -doteta*v_aux*np.exp(-eta)
    V_d    = v_aux*np.exp(-eta) + v_min
    
    f = (dotV_d + kf*(V_d-nv) + dp.T @ R @ mumat.T @ R.T @ dp/nv)/(ux.T @ R.T @ dp/nv)
    dotomega = ddtheta + hatchip.T @ E @ dp/nv - ktau*(omega-dtheta)
    tau = mutau*omega + dotomega
    
    return V,orierr,dtheta,tau, f
    