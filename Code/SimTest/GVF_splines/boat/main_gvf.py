import numpy as np
import matplotlib.pyplot as plt
import scipy as scp

from gvf_param_boat import gvf_speed_control
from gvf_param_boat import vector_field
from boat_model import boat_model
from matplotlib.path import Path
import matplotlib.patches as patches
 
# Por si se quiere guardar la animacion
animate = 0;

# Max simulation time (sec)
Time_max = 290; 


# Para seguir splines
t = np.array([0, 10, 20, 30, 40,  60,  80,  100, 120, 140, 160, 180, 200, 220, 240, 260])
x = np.array([-10, 13, 4, 22, 50, 27,  2,    1,   -10, -30, -20, -10, -20, -30, -20, -10])
y = np.array([5,  -10, 8,  20, 12, 0.2, 40,   20, 15, 2, -15, 40, 50, 50, 30, 50])
x[6:] += -20;


sx = scp.interpolate.CubicSpline(t,x,bc_type="natural")
sy = scp.interpolate.CubicSpline(t,y,bc_type="natural")


plt.close('all')



# Initial position
pos0 = np.array([x[0]-10,y[0]])


# Initial conditions
#x0 = [vx,vy,rx,ry,omega,R00,R01,R10,R11,w]
phi_orig = np.pi/6;
R = np.array([[np.cos(phi_orig), -np.sin(phi_orig)],
              [np.sin(phi_orig),np.cos(phi_orig)]])

x0 = np.array([1, 1, pos0[0], pos0[1], 0, R[0,0], R[0,1], R[1,0], R[1,1], t[0]])

# Sample time
Ts = 0.05;

# Max iterations
Niter_max = Time_max/Ts
k = 0;
cnt_iter = 0

# In order to show the complete trajectory in each sample time
ppx = []; ppy = []

# In order to show the complete trajectory evaluated with the integrator
pxtotal = []; pytotal = []

# In order to show speeds
dpxnorm = []; dpynorm = []
dppx = []; dppy = []

# In order to show vector fields
chixTotal = []; chiyTotal = []; 

# Save complete time vector
tt        = []

# In order to show distance errors
phixtotal = []; phiytotal = []

# In order to show angular control action uu, 
# Force control action ff,
# extended dynamics ww,
# Lyapunov function VV,
# vector field rotation rate dtheta,
# boats angular speed omegaomega
tautau = []; ff = []; ww = []; 
VV = []; dtheta = []; omegaomega = []

# Thrusters
TTp       = []; TTs       = []

# Orientation error
orierr    = []


# Vertices to draw the boat
factor = 0.6 # factor escalado para pintar barco
vertices = np.array([[0, 0.25],[0,-0.25],[1,0],[0,0.25]])*factor
orig_vertices = (R @ vertices.T).T 
vertices = orig_vertices + np.array([pos0[0],pos0[1]])
codes = [Path.MOVETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY]


################ Controller constants #############


"""
 kx : Vector field agressiveness in x direction
 ky : Vector field agressiveness in y direction
 ktau : Torque constant
 kf   : Force constant
"""
kx = 0.5; ky = 0.5; ktau = 10; kf = 1.1211

# Model parameters
f     = 0    # Initial force

mutau = 10    # Angular drag coef
mux   = 10    # Drag coef in x direction
muy   = 0.7   # Drag coef in y direction
mu    = np.array([[mux, 0], [0, muy]]) # Drag coef matrix
waterspeed = np.array([-0.1,0.1])*0    # Water speed (constant)

ux = np.array([1,0])                   # Unitary (body) [1 0] vector




# Simulation Loop
while((k < Niter_max)):

    
    ###### Get states
    #speed
    dpx     = x0[0]
    dpy     = x0[1]
    v       = np.array([dpx,dpy])
    #pos
    px     = x0[2]
    py     = x0[3]
    #angular speed
    omega  = x0[4]
    #Rotation matrix
    R00 = x0[5]
    R01 = x0[6]
    R10 = x0[7]
    R11 = x0[8]
    R = np.array([[R00,R01],[R10,R11]])
    #augmented dynamics
    w   = x0[9]
        
    """ Append section: """
    ###### get vertices
    newvertices = (R @ orig_vertices.T).T
    vertices = np.column_stack([vertices, newvertices + np.array([px,py])])

    ###### Add position, omega and w 
    ppx = np.append(ppx, px); ppy = np.append(ppy, py)
    omegaomega = np.append(omegaomega,omega)
    ww = np.append(ww,w)

    ###### Add speeds
    # Save normalize speeds
    dpxnorm = np.append(dpxnorm, dpx/np.sqrt(dpx**2 + dpy**2)) 
    dpynorm = np.append(dpynorm, dpy/np.sqrt(dpx**2 + dpy**2))
    # Non normalize speeds
    dppx = np.append(dppx,dpx)
    dppy = np.append(dppy,dpy)
    
    ###### Add vector field
    chi,chip,phix,phiy = vector_field([px,py], kx, ky, sx, sy, w)
    chin = chip/np.linalg.norm(chip)
    chixTotal = np.append(chixTotal, chin[0])
    chiyTotal = np.append(chiyTotal, chin[1])


    ###### Accelerations
    dv = (R @ ux)*f - (R @ mu @ R.T @ (v-waterspeed))
    
    #### Add phi errors
    phixtotal = np.append(phixtotal, phix)
    phiytotal = np.append(phiytotal, phiy)
    
    #### Add time 
    tt = np.append(tt, k*Ts)
    """ end of append section """
    
    
    #### Angular and speed control law
    V,e,dth,tau,f = gvf_speed_control([px,py], [dpx,dpy], dv, 
                                    R00,R01,R10,R11,omega,mutau,
                                    mu,kx, ky, ktau, kf, sx, sy, w,
                                    0,0,0)
    
    """ Append """
    #### Add orientation error, control action, Lyapunov and desired ang speed
    orierr = np.append(orierr,e)           # orientation error
    tautau = np.append(tautau,tau)         # Torque control action
    VV = np.append(VV,V)                   # Lyapunov function
    dtheta = np.append(dtheta,dth)         # Desired angular speed
    ff = np.append(ff,f)                   # Force control action
    
    #### Add thrusts
    TTp  = np.append(TTp,(f+tau)/2); TTs = np.append(TTs,(f-tau)/2)
    """ end of append """
    
    """ Actual simulation """
    #### Integrate
    results = scp.integrate.solve_ivp(boat_model, [k*Ts, (k+1)*Ts], 
                                 x0, dense_output=True,
                                 args=(f, tau, chi, mu, mutau,waterspeed),
                                 max_step = Ts)
    
    x0 = results.y;
    
    #### Get total position with better accuracy (from runge kutta integrator)
    pxtotal = np.append(pxtotal, results.y[2,:])
    pytotal = np.append(pytotal, results.y[3,:])
    
    x0 = x0[:,-1]
    k += 1
    

######### Plots #########

### Para mostrar trayectoria ###
plt.figure(figsize=(9,7))
tsp = np.linspace(0,t[-1],2000)
plt.plot(sx(tsp), sy(tsp), color = 'orange')
plt.plot(ppx, ppy, '--b')
plt.xlabel('x(t)'); plt.ylabel('y(t)'); plt.grid(True)

### Show initial point and splines control points ###
plt.plot(pos0[0], pos0[1], 'ko')
plt.scatter(x,y, s=15, color='red');
plt.legend(['Trayectoria deseada', 'trayectoria seguida', 'punto inicial', 
            'Puntos de ctrl splines'])

# Para el barco 
for k in range(1, int((vertices.shape[1])/2)-20, 10):
    vertex = vertices[:,2*k:2*k+2]# + np.array([ppx[k+1],ppy[k+1]])
    #plt.plot(ppx[k-1],ppy[k-1],'k.')
    pathi  = Path(vertex,codes)
    patchi = patches.PathPatch(pathi,facecolor = 'green')
    plt.gca().add_patch(patchi)
plt.gca().axis('equal')

### Comparamos la velocidad actual en cada Ts con la deseada ###
plt.figure(figsize=(9,7))

plt.subplot(211)
plt.plot(tt, dpxnorm)
plt.plot(tt, chixTotal) 
plt.legend(['Velocidad x', 'Velocidad deseada x'])
plt.title("Normalized speeds")
plt.grid(True)

plt.subplot(212)
plt.plot(tt, dpynorm)
plt.plot(tt, chiyTotal)
plt.legend(['Velocidad y', 'Velocidad deseada y'])
plt.xlabel("Time (s)")
plt.grid(True)

## Plot speed
plt.figure(figsize=(9,7))
plt.subplot(311)
plt.title("Actual speed")
plt.plot(tt,dppx,label=r"$\dot{x}(t)$")
plt.legend(fontsize=22)
plt.subplot(312)
plt.plot(tt,dppy,label=r"$\dot{y}(t)$")
plt.legend(fontsize=22)
plt.subplot(313)
plt.plot(tt,np.sqrt(dppy**2+dppx**2),label=r"$||\dot{r}||$")
plt.legend(fontsize=22)
#plt.plot(tt,np.sqrt(sx.derivative(1)(tt)**2 + sy.derivative(1)(tt)**2),'red',
#         label=r'$||\dot{\sigma(t)||$')

#### Mostrar curvas de nivel phi_i = 0
plt.figure(figsize=(9,7))
plt.subplot(121)
plt.plot(tt, phixtotal)
plt.title(r"Distance error $\phi_x$ (m)");
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(122)
plt.plot(tt,phiytotal)
plt.title(r"Distance error $\phi_y$ (m)");
plt.xlabel("Time (s)")
plt.grid(True)

plt.figure(figsize=(9,7))
plt.plot(tt, ww)
plt.title("w(t)"); plt.ylabel("w(t)")
plt.xlabel("Time (s)")
plt.grid(True)

## Plot respecto a w (3d dim)
ax = plt.figure(figsize=(9,7)).add_subplot(projection='3d')
ax.plot(ppx,ppy,ww)
ax.set_xlabel(r"$s_x(t)$"); 
ax.set_ylabel(r"$s_y(t)$"); 
ax.set_zlabel(r"$s_w(t)$")
ax.grid(True); ax.set_title(r"$(s_x(t),s_y(t),s_w(t))$")

# Plot ctrl actions
plt.figure(figsize=(9,7))
plt.subplot(311)
plt.title("Control actions")
plt.plot(tt,TTp, label='$T_p$')
plt.plot(tt,TTs, label='$T_s$')
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(312)
plt.plot(tt,ff, label=r'$f$')
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(313)
plt.plot(tt,tautau, label=r'$\tau$')
plt.xlabel("Time (s)")
plt.grid(True)
plt.legend(fontsize=22); plt.grid(True)

# Plot lyapunov function and its dependencies
plt.figure(figsize=(9,7))
plt.subplot(311)
plt.plot(tt,VV, label="Lyapunov function")
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(312)
#plt.plot(tt,omegaomega-dtheta, label=r"$\omega-\dot{\Theta}_d$")
plt.plot(tt,dtheta, label=r"$\omega-\dot{\Theta}_d$")
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(313)
plt.plot(tt,orierr/2,label=r"$(1-\hat{{\chi}^p}^TRu_x)$")
plt.legend(fontsize=22)
plt.xlabel("Time(s)")
plt.grid(True)

# Plot 
"""
plt.figure(figsize=(9,7))
plt.subplot(211)
plt.plot(tt,dtheta,label="$\dot{\Theta}_d$")
plt.plot(tt,omegaomega,label="$\omega$")
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(212)
plt.plot(tt,orierr/2,label=r"$(Ru_x - \hat{\chi}^{p})^T(Ru_x - \hat{\chi}^{p})/2$")
plt.xlabel("Time(s)")
plt.legend(fontsize=22); plt.grid(True)
plt.grid(True)
"""

## Animar
if(animate):
    from matplotlib.animation import FuncAnimation
    
    fig, ax = plt.subplots(figsize=(9,7))
    ax.set_xlabel("x(t)"); ax.set_ylabel("y(t)"); ax.grid(True)

    
    # Para crear animación
    def animate_uav(time):
    
        ax.clear()
        tsp = np.linspace(0,t[-1],1000)
        ax.plot(sx(tsp), sy(tsp), 'k', label='s(t) deseada')
        ax.plot(pxtotal, pytotal, '--b', label='s(t) seguida')
        ax.plot(ppx[10*(time)],ppy[10*(time)], 'ro', label='Boat')
        
        px = np.linspace(np.min(pxtotal)-40, np.max(pxtotal)+40, 25)
        py = np.linspace(np.min(pytotal)-40, np.max(pytotal)+40, 25)
    
        [Px,Py] = np.meshgrid(px,py)
        pdx = np.zeros(Px.shape)
        pdy = np.zeros(Py.shape)
    
        #vector_field([px,py], kx, ky, sx, sy, w)
    
        for k in range(len(px)):
            for u in range(len(px)):
                chi,chip,phix,phiy = vector_field([Px[k,u], Py[k,u]], kx, ky,
                                                  sx, sy, 
                                                  ww[10*(time)])
                #pd = pd/np.linalg.norm(pd)  # Normalizamos just in case
                pdx[k,u] = chi[0]
                pdy[k,u] = chi[1]
                    
        ax.quiver(Px, Py, pdx, pdy, color='red',label='Vector Field')
            
        # También plotear velocidad en cada instante (con el puntito)
        ax.quiver(ppx[10*time], ppy[10*time], 
                   dpxnorm[10*time], dpynorm[10*time] ,color='black',
                   label='Speed')
        
        ### Show initial point and splines control points ###
        ax.plot(pos0[0], pos0[1], 'ko',label='Initial Point')
        #ax.plot(x,y,'go')
        
        
        ax.legend()
    
    anim = FuncAnimation(fig, animate_uav, frames=int(len(ppx)/10 - 2), interval=1, repeat=False)
    anim.save('BOAT.gif')
    