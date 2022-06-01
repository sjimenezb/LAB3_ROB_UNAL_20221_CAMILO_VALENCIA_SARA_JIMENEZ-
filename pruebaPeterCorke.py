#
#import roboticstoolbox as rtb
#robot = rtb.models.DH.Panda()
#print(robot)
from cmath import nan, pi
from spatialmath.base import *
import math
import numpy as np
import types
import cmath
X = -32
Y = 0
Z = 347
anguloY=0 #aqui se la pose en grados del eje Y
pitchY = 0
Tt = transl(X,Y,Z)@ troty(pitchY)@ trotz(math.atan2(Y,-X))
print(Tt)


#function q_inv = invKinPxC(varargin)
aa=math.atan(100/32)*pi/180
off=[pi,aa,(pi/2)-aa,0]
T=Tt
l=[47,math.sqrt(10000+(32*32)),100,100]
#print(off)

#Desacople
Ph=np.array([[0,0,-100, 1]]).T
#Pb=np.dot(T,Ph)
Pb= T @ Ph
print(Pb)


#Theta 1
theta1 = math.atan2(Pb[1],-Pb[0])

#Theta 3
cos3  = (Pb[0]*Pb[0] + Pb[1]*Pb[1] +((Pb[2]-l[0])*(Pb[2]-l[0]))-l[1]*l[1]-l[2]*l[2])/(2*l[1]*l[2])
print("esto es cos3: ",cos3)
sin3D = cmath.sqrt(1-(cos3*cos3))#Codo Abajo es la raíz positiva
print("esto es csin3D: ",np.imag(sin3D))
sin3U = -cmath.sqrt(1-(cos3*cos3))#Codo Arriba es la raíz negativa

print(isinstance(sin3D,complex))
if np.imag(sin3D) == 0 :
    print("entré")
#Codo Abajo
    theta3D = math.atan2(np.real(sin3D),cos3)-off[2]
#Codo Arriba
    theta3U = math.atan2(np.real(sin3U),cos3)-off[2]

#Theta 2
    k1 = l[1]+l[2]*cos3
    k2 = l[2]*np.real(sin3D)
#Codo Abajo
    theta2D = (math.atan2(Pb[2]-l[0],math.sqrt(Pb[0]*Pb[0]+Pb[1]*Pb[1])) - math.atan2(k2,k1))-off[1]#; %Codo abajo es resta
#Codo Arriba
    theta2U = (math.atan2(Pb[2]-l[0],math.sqrt(Pb[0]*Pb[0]+Pb[1]*Pb[1])) + math.atan2(k2,k1))-off[1]#; %Codo arriba es suma
    #print(np.array(rotz(-theta1)).T)
    #print(T[0:3,0:3])
#Theta 4
    Rp = np.array(rotz(-theta1)).T @ T[0:3,0:3]
    pitch = math.atan2(Rp[2][0],Rp[0][0])
    theta4D = (pitch - theta2D - theta3D)
    theta4U = (pitch - theta2U - theta3U)
else :
   theta2D = float(nan)
   theta3D = float(nan)
   theta4D = float(nan)
   theta2U = float(nan)
   theta3U = float(nan)
   theta4U = float(nan)

q_invd=[0,0,0,0]
q_invu=[0,0,0,0]
q_invd = [theta1,theta2D,theta3D, theta4D]
q_invu = [theta1,theta2U,theta3U, theta4U]
 
print(q_invu)







