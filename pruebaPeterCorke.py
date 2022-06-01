#
#import roboticstoolbox as rtb
#robot = rtb.models.DH.Panda()
#print(robot)
from spatialmath.base import *
import math
X = -32
Y = 0
Z = 347
anguloY=0 #aqui se la pose en grados del eje Y
pitchY = deg2rad(anguloY) 
Tt = transl(X,Y,Z)@ troty(pitchY)@ trotz(math.atan2(Y,-X))
print(Tt)