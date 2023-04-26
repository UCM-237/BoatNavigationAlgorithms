import numpy as np
import matplotlib.pyplot as plt
import scipy as scp

from gvf_ctrl_spline import u_ctrl
from gvf_ctrl_spline import vector_field
from UAV_model import UAV_model

# Por si se quiere guardar la animaciñno
animate = 0;

# Max simulation time (sec)
Time_max = 390; 


# Para seguir splines
t = [1, 10, 20, 30, 40,  60,  80,  100, 120, 140, 160, 180, 200, 220, 240, 260]
x = [10, 13, 20, 22, 50, 27,  2,    1,   -10, -30, -20, -10, -20, -30, -20, -10]
y = [5,  10, 8,  20, 12, 0.2, 40,   20, 15, 2, -15, 40, 50, 50, 30, 50]

x = np.array(x)
x[6:] += -20;

sx = scp.interpolate.CubicSpline(t,x,bc_type="natural")
sy = scp.interpolate.CubicSpline(t,y,bc_type="natural")


plt.close('all')


# velocidad respecto al aire (fija por ahora a un valor bajo)
airspeed = 1

# Posición inicial
pos0 = np.array([0,-20])


# Condiciones inciales
x0 = np.array([pos0[0],pos0[1],np.pi/2,t[0]])

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


##### constantes de la ley de control
k1 = 0.05; k2 = 0.05; Kyaw = 3

plt.figure(figsize=(9,7))

while((k < Niter_max)):

    
    
    # Posiciones actuales, angulo actual, w actual
    px  = x0[0]; py = x0[1]; psi = x0[2]; w = x0[3]
    
    ppx = np.append(ppx, px); ppy = np.append(ppy, py)
    
    # third dimension (fake w = t, but w != t_{simu}, w será el tiempo
    # "interno" del Rover. Necesario que el rover use w como su tiempo.
    # Eso realmente es bueno, ya que así no hay que depender de un tiempo
    # externo, si no del w interno, el Rover llegará al final del spline
    # cuando w sea el que tiene que ser para el final del spline.)
    ww = np.append(ww,w)
    
    # Velocidades axtuales
    dpx = airspeed*np.cos(psi); dpy = airspeed*np.sin(psi)
    
    # Guardamos las velocidades para compararlas con la deseada
    dpxtotal = np.append(dpxtotal, dpx/airspeed) # normalizadas
    dpytotal = np.append(dpytotal, dpy/airspeed)
    
    # Guardar velocidad la deseada para comparar con la actual.
    # Necesario pasarle w claro !!!, dependen de w, w ~ t, pero no t de la simu
    # si no su t interna
    chi,chip,phi1,phi2 = vector_field([px,py], [dpx,dpy], airspeed, psi, k1, k2,
                             Kyaw, sx, sy, w)
    
    chin = chip/np.linalg.norm(chip)
    chixTotal = np.append(chixTotal, chin[0])
    chiyTotal = np.append(chiyTotal, chin[1])
    
    phi1total = np.append(phi1total, phi1)
    phi2total = np.append(phi2total, phi2)
    
    # Guardamos vector de tiempos
    tt = np.append(tt, k*Ts)
        
    # Ley de control. Le pasamos w claro
    u = u_ctrl([px,py], [dpx,dpy], airspeed, psi, k1, k2, Kyaw, sx, sy, w)
    
    # Simulación
    results = scp.integrate.solve_ivp(UAV_model, [k*Ts, (k+1)*Ts], 
                                 x0, dense_output=True,
                                 args=(airspeed, u,chi),
                                 max_step = 0.01)
    
    x0 = results.y;
    
    pxtotal = np.append(pxtotal, results.y[0,:])
    pytotal = np.append(pytotal, results.y[1,:])
    
    x0 = x0[:,-1]
    k += 1
    
######### Plots #########

### Para mostrar trayectoria ###
tsp = np.linspace(0,t[-1],1000)
plt.plot(sx(tsp), sy(tsp), color = 'orange')
plt.plot(pxtotal, pytotal, '--b')
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
plt.title("Velocidades normalizadas")
plt.grid(True)

plt.subplot(212)
plt.plot(tt, dpytotal)
plt.plot(tt, chiyTotal)
plt.legend(['Velocidad y', 'Velocidad deseada y'])
plt.xlabel("Time (s)")
plt.grid(True)


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

# Representamos la variable w respecto al tiempo
# Ver como al principo w ~ cte, ya que no está
# en la trayectoria, luego una vez esté en la
# trayectoria, entonces es cuando w empieza a crecer.
# w es el tiempo interno del rover. Incluso decrementa para que lueg
# tengamso el w en los valores de t_interno bien

plt.figure(figsize=(9,7))
plt.plot(tt, ww)
plt.title("w(t)"); plt.ylabel("w(t)")
plt.xlabel("Time (s)")
plt.grid(True)

## Plot respecto a w (3d dim)
ax = plt.figure(figsize=(9,9)).add_subplot(projection='3d')
ax.plot(ppx,ppy,ww)
ax.set_xlabel("$s_x(t)$",fontsize=22); 
ax.set_ylabel("$s_y(t)$",fontsize=22); 
ax.set_zlabel("$s_w(t)$",fontsize=22)
ax.grid(True); ax.set_title("$(s_x(t),s_y(t),s_w(t))$",fontsize=22)


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
        ax.plot(ppx[10*(time)],ppy[10*(time)], 'ro', label='Rover')
        
        px = np.linspace(np.min(pxtotal)-9, np.max(pxtotal)+9, 25)
        py = np.linspace(np.min(pytotal)-9, np.max(pytotal)+9, 25)
    
        [Px,Py] = np.meshgrid(px,py)
        pdx = np.zeros(Px.shape)
        pdy = np.zeros(Py.shape)
    
        for k in range(len(px)):
            for u in range(len(px)):
                chi,chip,phi1,phi2 = vector_field([Px[k,u], Py[k,u]], [0,0],
                                                  airspeed, 0, k1, k2,
                                                  Kyaw, sx, sy, 
                                                  ww[10*(time)])
                #pd = pd/np.linalg.norm(pd)  # Normalizamos just in case
                pdx[k,u] = chi[0]
                pdy[k,u] = chi[1]
                    
        ax.quiver(Px, Py, pdx, pdy, color='red',label='Vector Field')
            
        # También plotear velocidad en cada instante (con el puntito)
        ax.quiver(ppx[10*time], ppy[10*time], 
                   dpxtotal[10*time], dpytotal[10*time] ,color='black',
                   label='Speed')
        
        ### Show initial point and splines control points ###
        ax.plot(pos0[0], pos0[1], 'ko',label='Initial Point')
        #ax.plot(x,y,'go')
        
        
        ax.legend()
    
    anim = FuncAnimation(fig, animate_uav, frames=int(len(ppx)/10 - 2), interval=1, repeat=False)
    anim.save('UAV.gif')
    