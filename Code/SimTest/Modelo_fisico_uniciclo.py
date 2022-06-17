# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 19:07:11 2022
Modelito elemental de la boya. Lo que cuenta es el guiado
@author: abierto
"""

import numpy as np
import matplotlib.pyplot as pl
from matplotlib.path import Path
import matplotlib.patches as patches
import gvf
import math

#Costantes del modelo absolutamente arbitrarias
#matriz de rozamientos en ejes cuerpo. Incluyen todo Inercias, masas ..
#mu = np.array([[10,0],[0,100]])
mua = 0. #resistencia al giro 
ang_max= 0.10 #angulo maximo de giro del robot en radianes
#parametros del controlador
ke = 0.4
            

tfin = 100#tiempo final del experimento
t = 0. #tiempo actual
dt = 0.001 #incremento de tiempos para integración

#parametros de la elipse a seguir (gran guarrada)
a = 6 #semieje x
b = 5 #semieje y
alpha = 0 #orientación de la elipse
p0 = np.array([[0],[0]]) #centro de la elipse


tp = tfin/40#tiempo de pintar
p = np.array([[15.],[20.]]) #posición actual
phi=1.5; #orientacion inicial
m = np.array([[math.cos(phi)], [math.sin(phi)]]) #vector de rumbo
s=0.5;  #velocidad
#wt = np.array([[-0.3],[-0.3]]) #velocidad del agua, Camarón que se duerme...
#supongo coordenadas NED: los pingüinos siempre miran hacia el norte
R = np.eye(2)
phidot_max=2 #radianes por segundo, velocidad maxima de giro
kd=4*(a/s)*(phidot_max-1)
#ojo que esta E, es la traspuesta de la de Héctor
E = np.array([[0,-1],[1,0]])

w = 0. #velocidad angular del bicho
Sw = E*w

#Td = 0. #fuerza de los thrusters
#Ti = 0.

#Velocidad inicial, no parto del reposo porque gvf, necesita un p_dot distinto
#de cero
ux = np.array([[1],[0]])

#pa pintar los resultados
vertices = np.array([[0.1, 0],[-0.1,0],[0,0.2],[0.1,0]])
codes = [Path.MOVETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY]

#bucle de integración
#F = 10 #fuerza (fija la velocidad de consigna)
#v = np.array([[0.],[F/mu[0,0]]]) #velocidad actual
v=np.array([[0],[1.0]])

Tau = 0 #par
F=10;
pl.figure(2)
pl.axis('equal')

pl.figure(6)
pl.plot(-p[1,0],p[0,0],'rx')
pl.arrow(-p[1,0],p[0,0],-m[1,0],m[0,0], width=0.01,head_width=0.1,ec='green')
#prueba con el control de héctor para circular
#circulo = gvfH.Path_gvf_circle(p0[0][0],p0[1][0], 2)
while t <= tfin:
    #de momento solo controlamos rumbo y dejamos la v fija como partimos del
    #reposo deberíamos dejar que coja velocidad antes de lanzar el algoritmo..
    
    #e,n,H = gvf.elipse(a, b, alpha, p, p0)
    e,n,H = gvf.circulo(p,p0,a)
    #m = R[:,1].reshape(2,1)
    Tau = gvf.gvf_control_boat(p, v, e, n, H, ke, kd, s, m)
    
    #Td = (F + Tau)/2
    #Ti = (F - Tau)/2
    #v += (R.dot(ux*(Td+Ti)) - R.dot(mu.dot(R.T.dot(v - wt))))*dt
    phi+=(Tau)*dt;

    while phi >=math.pi:
        phi = phi - 2*math.pi
    while phi < -(math.pi):
        phi= 2*math.pi + phi
 
    m=np.array([[math.cos(phi)],[math.sin(phi)]])
    v = s*m;
    p += v*dt;
    #p +=s*m*dt+w*dt
    #v += s*m+w

#     pdot = s*m;
#     p += v*dt
    w += (Tau - mua*w)*dt #desprecio cualquier par que pueda hacer la corriente
    Sw = E*w 
    R += R.dot(Sw)*dt

    # TEST
    if t <= 0.5:
        pl.figure(7)
        pl.plot(t,v[0],'.r')
        pl.plot(t,v[1],'.k')
        pl.plot(t,w,'.b')
        pl.figure(8)
        pl.plot(-p[1],p[0],'.')
        pl.figure(9)
        pl.plot(t,phi,'.b')


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
        pl.figure(3)
        pl.plot(t,Tau,'.b')
        #pl.plot(t,Tau2,'.y')
        #pl.plot(t,circulo.e,'*k')
        pl.figure(4)
        pl.plot(t,e,'.r')
        pl.figure(5)
        pl.plot(t,phi,'.b')
        pl.figure(6)
        pl.plot(-p[1],p[0],'.')
        pl.arrow(-p[1,0],p[0,0],-m[1,0],m[0,0], width=0.01,head_width=0.1)
        tp += tfin/40

    t = t + dt
pl.figure(1)
pl.legend(['vx','vy','w'])
pl.figure(3)
pl.legend(['Control'])
pl.figure(4)
pl.legend(['Error'])
pl.figure(5)
pl.legend(['Phi'])

pl.show()
#pl.arrow(-p[1],p[0],v[1],v[0]) 

