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

def simula(theta,a,b,p0):
    #Costantes del modelo absolutamente arbitrarias
    #matriz de rozamientos en ejes cuerpo. Incluyen todo Inercias, masas ..
    mux=10
    muy=100
    mu = np.array([[mux,0],[0,muy]])
    mua = 10. #resistencia al giro 

    #parametros del controlador
    ke = 0.8
    kd = 10           

    tfin = 10#tiempo final del experimento
    t = 0. #tiempo actual
    dt = 0.001 #incremento de tiempos para integración

    #parametros de la elipse a seguir (gran guarrada)
    r=2
    alpha = 0 #orientación de la elipse

    tp = tfin/40 #tiempo de pintar
    p = np.array([[5.4],[8.1]]) #posición actual

    wt = np.array([[-0.3],[-0.3]]) #velocidad del agua, Camarón que se duerme...
    #supongo coordenadas NED: los pingüinos siempre miran hacia el norte
    R = -np.eye(2)
    #ojo que esta E, es la traspuesta de la de Héctor
    E = np.array([[0,1],[-1,0]])


    w = 0. #velocidad angular del bicho
    Sw = E*w

    Td = 0. #fuerza de los thrusters
    Ti = 0.

    #Velocidad inicial, no parto del reposo porque gvf, necesita un p_dot distinto
    #de cero
    ux = np.array([[1],[0]])


    #bucle de integración
    F = 10 #fuerza (fija la velocidad de consigna)
    v = np.array([[0.],[F/mu[0,0]]]) #velocidad actual
    Tau = 0 #par


    v_deseada_x = np.array([0.0])
    v_deseada_y = np.array([0.0])
    v_real_x = np.array([0.])
    v_real_y= np.array([F/mu[0,0]])
    t_array=np.array([0])
    p_x=np.array([-8.1])
    p_y=np.array([5.4])
    Tau_array=np.array([0.])
    Td_array=np.array([F/2])
    Ti_array=np.array([F/2])
    e_array=np.array([0.])
    dot_Xd_array=np.array([0.])
    kd_term=np.array([0.0])

    R = np.cos(theta)*R
    print(R)
    #prueba con el control de héctor para circular
    #circulo = gvfH.Path_gvf_circle(p0[0][0],p0[1][0], 2)
    while t <= tfin:
        #de momento solo controlamos rumbo y dejamos la v fija como partimos del
        #reposo deberíamos dejar que coja velocidad antes de lanzar el algoritmo..
    
        e,n,H = gvf.elipse(a, b, alpha, p, p0)
        #e,n,H = gvf.circulo(p,p0,2)
        m = R[:,1].reshape(2,1)
        Tau,ghi,dot_pdhat, dot_Xd = gvf.gvf_control_boat_2(p, v, e, n, H, ke, kd, 1, m,mua,10,100,F,R)
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
        t = t + dt
        t_array=np.append(t_array,[t],axis=0)
    return p_x,p_y,t_array 

a = 8#semieje x
b = 12 #semieje y
p0 = np.array([[0],[0]]) #centro de la elipse
vertices = np.array([[0.1, 0],[-0.1,0],[0,0.2],[0.1,0]])
codes = [Path.MOVETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY]
theta_array = np.linspace(0,np.pi,4)
for theta in theta_array:
    p_x,p_y,t_array=simula(theta,a,b,p0)
    pl.figure(1)
    pl.axis('equal')
    pl.plot(p_x,p_y,'-')
    R=(-np.eye(2))*np.cos(theta)
    vertrot = np.array([np.dot(R,j) for j in vertices]) +\
    [p_x[0],p_y[0]]       
    pathi = Path(vertrot,codes)
    patchi = patches.PathPatch(pathi)
    pl.gca().add_patch(patchi)    



num_puntos=100
x = np.linspace(-b,b, num_puntos)
pl.plot(x, fe(x,a,b,p0), color="red", markersize=1)
# dibuja los puntos x,y calculados
pl.plot(x, -fe(x,a,b,p0), color="red", markersize=1)
pl.title(['Posiciones'])
pl.show()

    #pl.title('ke=0.8,kd=10,p_inicial=[0.4],[4.9]]')