# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 19:07:11 2022
Modelito elemental de la boya. Lo que cuenta es el guiado
@author: abierto
"""

import numpy as np
import numpy.linalg as npl
import matplotlib.pyplot as pl
from matplotlib.path import Path
import matplotlib.patches as patches
import gvf
from matplotlib.patches import Rectangle, Arrow

def f(x,a,b,r):
    y = np.sqrt(r**2 - ((x - a) ** 2))
    return y
def fe(x,a,b,p0):
    y = np.sqrt(1-(x-p0[0])**2/b**2)*a+p0[1]
    return y


#Costantes del modelo absolutamente arbitrarias
#matriz de rozamientos en ejes cuerpo. Incluyen todo Inercias, masas ..
mux=10
muy=100
mu = np.array([[mux,0],[0,muy]])
mua = 10. #resistencia al giro 

#parametros del controlador
ke = 0.8
kd = 10           

tfin = 50#tiempo final del experimento
t = 0. #tiempo actual
dt = 0.001 #incremento de tiempos para integración

#parametros de la elipse a seguir (gran guarrada)
a = 8#semieje x
b = 12 #semieje y
r=2
alpha = 0 #orientación de la elipse
p0 = np.array([[0],[0]]) #centro de la elipse


tp = tfin/40 #tiempo de pintar
p = np.array([[15.4],[18.1]]) #posición actual

wt = np.array([[-0.3],[-0.3]]) #velocidad del agua, Camarón que se duerme...
#supongo coordenadas NED: los pingüinos siempre miran hacia el norte
R = -np.eye(2)
theta = 1.2
R = np.cos(theta)*R
print(R)
#ojo que esta E, es la traspuesta de la de Héctor
E = np.array([[0,1],[-1,0]])


w = 0. #velocidad angular del bicho
Sw = E*w

Td = 0. #fuerza de los thrusters
Ti = 0.

#Velocidad inicial, no parto del reposo porque gvf, necesita un p_dot distinto
#de cero
ux = np.array([[1],[0]])

#pa pintar los resultados
vertices = np.array([[0.1, 0],[-0.1,0],[0,0.2],[0.1,0]])
codes = [Path.MOVETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY]

#bucle de integración
F = 10 #fuerza (fija la velocidad de consigna)
v = np.array([[0.],[F/mu[0,0]]]) #velocidad actual
Tau = 0 #par

pl.figure(2)
pl.axis('equal')
v_deseada_x = np.array([0.0])
v_deseada_y = np.array([0.0])
v_real_x = np.array([0.])
v_real_y= np.array([F/mu[0,0]])
t_array=np.array([0])
p_x=np.array([-18.1])
p_y=np.array([15.4])
Tau_array=np.array([0.])
Td_array=np.array([F/2])
Ti_array=np.array([F/2])
e_array=np.array([0.])
dot_Xd_array=np.array([0.])
kd_term=np.array([0.0])
#prueba con el control de héctor para circular
#circulo = gvfH.Path_gvf_circle(p0[0][0],p0[1][0], 2)
while t <= tfin:
    #de momento solo controlamos rumbo y dejamos la v fija como partimos del
    #reposo deberíamos dejar que coja velocidad antes de lanzar el algoritmo..
    
    e,n,H = gvf.elipse(a, b, alpha, p, p0)
    #e,n,H = gvf.circulo(p,p0,2)
    m = R[:,1].reshape(2,1)
    Tau,ghi,dot_pdhat, dot_Xd = gvf.gvf_control_boat_2(p, v, e, n, H, ke, kd, 1, m,mua,10,100,F,R)
    #print(Tau)
    Td = (F + Tau)/2
    Ti = (F - Tau)/2
    v += (R.dot(ux*(Td+Ti)) - R.dot(mu.dot(R.T.dot(v - wt))))*dt
    p += v*dt
    w += (Td-Ti - mua*w)*dt #desprecio cualquier par que pueda hacer la corriente
    Sw = E*w 
    R += R.dot(Sw)*dt
    #Guardo datos en array
    v_deseada_x=np.append(v_deseada_x,dot_pdhat[0],axis=0)
    v_real_x=np.append(v_real_x,v[0]/npl.norm(v),axis=0)
    v_deseada_y=np.append(v_deseada_y,dot_pdhat[1],axis=0)
    v_real_y=np.append(v_real_y,v[1]/npl.norm(v),axis=0)
    p_x=np.append(p_x,-p[1],axis=0)
    p_y=np.append(p_y,p[0],axis=0)
    Tau_array=np.append(Tau_array,Tau[0],axis=0)
    Td_array=np.append(Td_array,Td[0],axis=0)
    Ti_array=np.append(Ti_array,Ti[0],axis=0)
    e_array=np.append(e_array,[e],axis=0)
    dot_Xd_array=np.append(dot_Xd_array,dot_Xd[0],axis=0)
    kd_term=np.append(kd_term,dot_Xd[0]-Tau[0],axis=0)
    #solo pinto de vez en cuando
    if t>=tp:
        pl.figure(1)
        pl.plot(t,v[0],'.r')
        pl.plot(t,v[1],'.k')
        pl.plot(t,w,'.b')
        pl.figure(2)
        pl.plot(-p[1],p[0],'.')
        vertrot = np.array([np.dot(R,j) for j in vertices]) +\
        [-p[1,0],p[0,0]]       
        pathi = Path(vertrot,codes)
        patchi = patches.PathPatch(pathi,facecolor = 'blue')
        pl.gca().add_patch(patchi)
        #pl.arrow(-p[1],p[0],v[1],v[0]) #NED
        pl.figure(4)
        pl.plot(t,ghi,'.b')
#         pl.figure(5)
#         x=np.array([-p[1]],dtype=object)
#         y=np.array([p[0]],dtype=object)
#         z=np.array([-dot_pdhat[1]],dtype=object)
#         u=np.array([dot_pdhat[0]],dtype=object)
#         pl.quiver([-p[1,0],p[0,0]],[-dot_pdhat[1,0],dot_pdhat[0,0]])
#         pl.figure(6)
#         vhat=v/npl.norm(v)
#         pl.plot(t,-dot_pdhat[1,0],'b.-',linewidth=2)
#         pl.plot(t,-vhat[1],'ro-',linewidth=2)
#         pl.figure(7)
#         pl.plot(t,-dot_pdhat[0,0],'b.-',linewidth=2)
#         pl.plot(t,vhat[0],'r.-',linewidth=2)
        tp += tfin/40
    t = t + dt
    t_array=np.append(t_array,[t],axis=0)

pl.figure(1)
pl.legend(['vx','vy','w'])
pl.figure(2)
num_puntos=100
x = np.linspace(-b,b, num_puntos)
# dibuja el circulo con lineas cortas
pl.plot(x, fe(x,a,b,p0), color="red", markersize=1)
# dibuja los puntos x,y calculados
pl.plot(x, -fe(x,a,b,p0), color="red", markersize=1)
pl.plot(p_x,p_y,'k-')
#pl.title('ke=0.8,kd=10,p_inicial=[0.4],[4.9]]')
pl.legend(['Posiciones'])
pl.figure(6)
pl.plot(t_array,v_deseada_x,'r-')
pl.plot(t_array,v_real_x,'b-')
pl.legend(['dotpd_hat_x','v_x'])
pl.figure(7)
pl.plot(t_array,v_deseada_y,'k-')
pl.plot(t_array,v_real_y,'g-')
pl.legend(['dotpd_hat_y','v_y'])
fig, axs = pl.subplots(2)
axs[0].plot(t_array,dot_Xd_array,'b-')
pl.legend(['dot_Xd'])
axs[1].plot(t_array,kd_term,'g-')
pl.legend(['dot_Xd-Tau'])
fig, axs = pl.subplots(2)
axs[0].plot(t_array,Tau_array,'b-')
axs[0].plot(t_array,Td_array,'g-')
axs[0].plot(t_array,Ti_array,'k-')
pl.legend(['Tau','Td','Ti'])
axs[1].plot(t_array,e_array,'r-')
pl.legend(['Error'])
pl.show()
#pl.arrow(-p[1],p[0],v[1],v[0]) 


 