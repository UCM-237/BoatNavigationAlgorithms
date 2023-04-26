import numpy as np
import matplotlib.pyplot as plt
import scipy as scp

from gvf_param_boat import u_ctrl
from gvf_param_boat import vector_field
from boat_model import boat_model

# Por si se quiere guardar la animaciñno
animate = 1;

# Max simulation time (sec)
Time_max = 2300; 


# Para seguir splines
t = [0, 10, 20, 30, 40,  60,  80,  100, 120, 140, 160, 180, 200, 220, 240, 260]
x = np.array([-10, 13, 4, 22, 50, 27,  2,    1,   -10, -30, -20, -10, -20, -30, -20, -10])
y = np.array([5,  -10, 8,  20, 12, 0.2, 40,   20, 15, 2, -15, 40, 50, 50, 30, 50])
x[6:] += -20;

#t = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80])
#x = np.array([1, 10, 20, 30, 40, 50, 60, 70, 80])
#y = np.array([2,  2,  2,  2,  2,  2,  2,  2, 2])
#y = x
x = x*10; y = y*10; t = np.array(t)*10



sx = scp.interpolate.CubicSpline(t,x,bc_type="natural")
sy = scp.interpolate.CubicSpline(t,y,bc_type="natural")


plt.close('all')



# Posición inicial
pos0 = np.array([x[0]+90,y[0]-30])


# Condiciones inciales
#x0 = [vx,vy,rx,ry,omega,R00,R01,R10,R11,w]
x0 = np.array([1, 1, pos0[0], pos0[1], 0, 1, 0, 0, 1,t[0]])

# Tiempo muestreo
Ts = 0.1;

# Max iterations
Niter_max = Time_max/Ts
k = 0;
cnt_iter = 0

# Igual que el siguiente pero 1 por cada Ts
ppx = []
ppy = []

# Para luego representar trayectoria y campos vectoriales
pxtotal = []
pytotal = []

# Para leuego representar velocidades 
dpxtotal = []
dpytotal = []

dppx = []
dppy = []

# Para luego representar velocidades deseadas y guardar tiempo cada Ts
chixTotal = []
chiyTotal = []
tt        = []

# Para ver si converge
phi1total = []
phi2total = []

# Para guardar la ley de control u
uu        = []
ww        = []
VV        = []
dtheta    = []
omegaomega = []
TTp       = []
TTs       = []
orierr    = []
##### constantes de la ley de control

## Si hay disturbances subir la k1 y k2....
# Como hay rozamientos hay que subir las k1 k2
k1 = 1.8; k2 = 1.8; ktau = 12

## parametros del modelo
f     = 10

mutau = 10
mux   = 10
muy   = 0.3
mu    = np.array([[mux, 0], [0, muy]])
waterspeed = np.array([-0.6,0.5])

ux = np.array([1,0])

plt.figure(figsize=(9,7))

while((k < Niter_max)):

    
    
    # Posiciones actuales, angulo actual, w actual
    #speed
    dpx     = x0[0]
    dpy     = x0[1]
    v       = np.array([dpx,dpy])
    #pos
    px     = x0[2]
    py     = x0[3]
    #r      = np.array([rx,ry])
    #ang speed
    omega  = x0[4]
    
    # Elements of Rot Matrix
    R00 = x0[5]
    R01 = x0[6]
    R10 = x0[7]
    R11 = x0[8]
    R = np.array([[R00,R01],[R10,R11]])
    
    # augmented dynamics
    w   = x0[9]
    
    ppx = np.append(ppx, px); ppy = np.append(ppy, py)
    omegaomega = np.append(omegaomega,omega)
    ww = np.append(ww,w)
    
    # Velocidades axtuales
    
    # Guardamos las velocidades normalizadas para compararlas con la deseada
    dpxtotal = np.append(dpxtotal, dpx/np.sqrt(dpx**2 + dpy**2)) # normalizadas
    dpytotal = np.append(dpytotal, dpy/np.sqrt(dpx**2 + dpy**2))
    
    # Velocidades sin normalizar
    dppx = np.append(dppx,dpx)
    dppy = np.append(dppy,dpy)
    
    # Accelerations (not used yet)
    dv = (R @ ux)*f - (R @ mu @ R.T @ (v-waterspeed))
    ddp = dv;

    
    # Guardar velocidad la deseada para comparar con la actual.
    chi,chip,phi1,phi2 = vector_field([px,py], k1, k2, sx, sy, w)
    
    chin = chip/np.linalg.norm(chip)
    chixTotal = np.append(chixTotal, chin[0])
    chiyTotal = np.append(chiyTotal, chin[1])
    
    phi1total = np.append(phi1total, phi1)
    phi2total = np.append(phi2total, phi2)
    
    # Guardamos vector de tiempos
    tt = np.append(tt, k*Ts)
    
    
    # Ley de control. Le pasamos w claro
    V,e,dth,u = u_ctrl([px,py], [dpx,dpy], dv, R00,R01,R10,R11,omega,mutau,k1, k2, ktau, sx, sy, w)
    orierr = np.append(orierr,e) # orientation error
    uu = np.append(uu,u)         # Control action
    VV = np.append(VV,V)         # Lyapunov function
    dtheta = np.append(dtheta,dth) # Desired angular speed
    
    # Thrust
    TTp  = np.append(TTp,(f+u)/2); TTs = np.append(TTs,(f-u)/2)
    
    # Simulación
    results = scp.integrate.solve_ivp(boat_model, [k*Ts, (k+1)*Ts], 
                                 x0, dense_output=True,
                                 args=(f, u, chi, mu, mutau,waterspeed),
                                 max_step = Ts)
    
    x0 = results.y;
    
    pxtotal = np.append(pxtotal, results.y[2,:])
    pytotal = np.append(pytotal, results.y[3,:])
    
    x0 = x0[:,-1]
    k += 1
    
######### Plots #########

### Para mostrar trayectoria ###
tsp = np.linspace(0,t[-1],2000)
plt.plot(sx(tsp), sy(tsp), color = 'orange')
plt.plot(ppx, ppy, '--b')
plt.xlabel('x(t)'); plt.ylabel('y(t)'); plt.grid(True)

### Show initial point and splines control points ###
plt.plot(pos0[0], pos0[1], 'ko')
plt.scatter(x,y, s=15, color='red');
plt.legend(['Trayectoria deseada', 'trayectoria seguida', 'punto inicial', 
            'Puntos de ctrl splines'])

### Comparamos la velocidad actual en cada Ts con la deseada ###
plt.figure(figsize=(9,7))

plt.subplot(211)
plt.plot(tt, dpxtotal)
plt.plot(tt, chixTotal) 
plt.legend(['Velocidad x', 'Velocidad deseada x'])
plt.title("Normalized speeds")
plt.grid(True)

plt.subplot(212)
plt.plot(tt, dpytotal)
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

#### Mostrar curvas de nivel phi_i = 0
plt.figure(figsize=(9,7))

plt.subplot(121)
plt.plot(tt, phi1total)
plt.title("$\phi_1$ (m)");
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(122)
plt.plot(tt,phi2total)
plt.title("$\phi_2$ (m)");
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
plt.subplot(211)
plt.plot(tt,TTp, label='$T_p$')
plt.plot(tt,TTs, label='$T_s$')
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(212)
plt.plot(tt,uu, label=r'$\tau$')
plt.title("Control actions")
plt.xlabel("Time (s)")
plt.grid(True)
plt.legend(fontsize=22); plt.grid(True)

# Plot lyapunov function and its dependencies
plt.figure(figsize=(9,7))
plt.subplot(311)
plt.plot(tt,VV, label="Lyapunov function")
plt.legend(fontsize=22); plt.grid(True)
plt.subplot(312)
plt.plot(tt,omegaomega-dtheta, label=r"$\omega-\dot{\Theta}_d$")
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
        ax.plot(ppx[100*(time)],ppy[100*(time)], 'ro', label='Boat')
        
        px = np.linspace(np.min(pxtotal)-400, np.max(pxtotal)+400, 25)
        py = np.linspace(np.min(pytotal)-400, np.max(pytotal)+400, 25)
    
        [Px,Py] = np.meshgrid(px,py)
        pdx = np.zeros(Px.shape)
        pdy = np.zeros(Py.shape)
    
        #vector_field([px,py], k1, k2, sx, sy, w)
    
        for k in range(len(px)):
            for u in range(len(px)):
                chi,chip,phi1,phi2 = vector_field([Px[k,u], Py[k,u]], k1, k2,
                                                  sx, sy, 
                                                  ww[100*(time)])
                #pd = pd/np.linalg.norm(pd)  # Normalizamos just in case
                pdx[k,u] = chi[0]
                pdy[k,u] = chi[1]
                    
        ax.quiver(Px, Py, pdx, pdy, color='red',label='Vector Field')
            
        # También plotear velocidad en cada instante (con el puntito)
        ax.quiver(ppx[100*time], ppy[100*time], 
                   dpxtotal[100*time], dpytotal[100*time] ,color='black',
                   label='Speed')
        
        ### Show initial point and splines control points ###
        ax.plot(pos0[0], pos0[1], 'ko',label='Initial Point')
        #ax.plot(x,y,'go')
        
        
        ax.legend()
    
    anim = FuncAnimation(fig, animate_uav, frames=int(len(ppx)/100 - 2), interval=1, repeat=False)
    anim.save('BOAT.gif')
    