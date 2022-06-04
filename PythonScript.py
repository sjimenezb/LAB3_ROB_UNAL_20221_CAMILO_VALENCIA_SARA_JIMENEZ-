import termios, sys, os
import rospy
from geometry_msgs.msg import Twist
from numpy import pi

TERMIOS = termios
import time
from dynamixel_workbench_msgs.srv import DynamixelCommand
from spatialmath.base import *
import math

from cmath import nan, pi
import numpy as np
import types
import cmath
from operator import add

# Tener corriendo en un terminal el paquete "px_controllers.launch" del repositorio anterior

# Defino cómo variables globales las posiciones iniciales
X = -132
Y = 0
Z = 47
pitchY = pi
modoSelect = 0

q=[0,0,0,0]

# Crea el servicio dentro del nodo que creo en el controller.launch
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


# Función para detectar la tecla. Regresa la tecla presionada 
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

def movArt(q1):

    print("este es q1 en radianes: ", q1)
    #pasitos=[math.floor(((q1[0]*(180/pi))+180)*(4096/360)),math.floor(((q1[1]*(180/pi))+180)*(4096/360)),math.floor(((q1[2]*(180/pi))+270)*(4096/360)),math.floor(((q1[3]*(180/pi))+180)*(4096/360))]
    pasos=[math.floor(((q1[0]*(180/pi))+0)*(4096/360)),math.floor(((q1[1]*(180/pi))+180)*(4096/360)),math.floor(((q1[2]*(180/pi))+270)*(4096/360)),math.floor(((q1[3]*(180/pi))+0)*(4096/360))]
    print("este es pasos: ",pasos)

    jointCommand('', 1, 'Goal_Position', pasos[0], 0.05)
    jointCommand('', 2, 'Goal_Position', pasos[1], 0.05)
    jointCommand('', 3, 'Goal_Position', pasos[2], 0.05)
    jointCommand('', 4, 'Goal_Position', pasos[3], 0.05)
    print("movimiento exitoso")

def ModTt(modoSelect1,tecla2, X1, Y1, Z1, pitchY1):
    print("tu pose actual es: ", X1, Y1 , Z1 , pitchY1)
    cambio=5
    if modoSelect1 == 0 and tecla2 == 0: ##cuando a=0 es poque suma, a=1 resta
        print("estás modificando en trax + ")
        X1=X1+cambio      
    elif modoSelect1 == 0 and tecla2 == 1:
        print("estás modificando en trax - ")
        X1=X1-cambio
    elif modoSelect1 == 1 and tecla2 == 0:
        print("estás modificando en tray + ")
        Y1=Y1+cambio
    elif modoSelect1 == 1 and tecla2 == 1:
        print("estás modificando en tray - ")
        Y1=Y1-cambio
    elif modoSelect1 == 2 and tecla2 == 0:
        print("estás modificando en traz + ")
        Z1=Z1+cambio
    elif modoSelect1 == 2 and tecla2 == 1:
        print("estás modificando en traz - ")
        Z1=Z1-cambio
    elif modoSelect1 == 3 and tecla2 == 0:
        print("estás modificando rot + ")
        pitchY1=pitchY1+0.1
    elif modoSelect1 == 3 and tecla2 == 1:
        print("estás modificando rot - ")
        pitchY1=pitchY1-0.1
    
    print("la nueva pose es: ", X1, Y1 , Z1 , pitchY1)
    return X1, Y1 , Z1 , pitchY1

def generarTt(X, Y , Z , pitchY):
    Tt = transl(X,Y,Z)@ troty(pitchY)@ trotz(math.atan2(Y,-X))
    print("Esto es Tt: ",Tt)
    return(Tt)

def invkinPxC(Tttemp):
    print("llegue hasta generar los q")
    aa=np.arctan(100/32)*pi/180
    #off=[pi,aa,(pi/2)-aa,0]
    off=np.array([0,aa,0,0])
    T=Tttemp
    l=np.array([47,math.sqrt(100*100+(32*32)),100,100])
    print("Esto son offsets:", off)
    print("Esto son longitudes:", l)
    #Desacople
    Pb = T-(l[3]*T[0:4,2]).reshape(4,1)
    print("este es pb: ",Pb)
    #Theta 1
    theta1 = np.arctan2(Pb[1,3],Pb[0,3])
    #Solución mediante análisis geométrico 2R
    h = Pb[2,3]-l[0]
    r = np.sqrt(Pb[0,3]**2 + Pb[1,3]**2)
    #Solución codo abajo:
    theta3D = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    theta2D = -(pi/2-off[1]- (np.arctan2(h,r) - np.arctan2(l[2]*np.sin(theta3D),l[1]+l[2]*np.cos(theta3D))))
    #Solución codo arriba:
    theta2U = -(pi/2-off[1]- (np.arctan2(h,r) + np.arctan2(l[2]*np.sin(theta3D),l[1]+l[2]*np.cos(theta3D))))
    theta3U = -theta3D
    #Solución Theta 4
    Rp = (rotz(theta1)).T.dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    theta4D = (pitch - theta2D - theta3D)
    theta4U = (pitch - theta2U - theta3U)
    if theta4U > (7/6)*np.pi:
        theta4U = theta4U -2*np.pi
    if theta4D > (7/6)*np.pi:
        theta4D = theta4D -2*np.pi
    q_inv = np.empty((1,4))
    q_inv[:] = np.NaN
    q_inv = [theta1, theta2U, theta3U, theta4U]
    return q_inv

if __name__ == '__main__':
    try:
        print('Se encuentra en la trax presione una W o S para adelantar o regresar y A o D para cambiar entre posiciones')
        while True:
            print("presione letra: ")
            letra=getkey()        
            print('\n')
            print('la tecla presioada fue: ')
            print(letra)

            if letra == b'w' or letra == b'W':
                print("seleccionaste w")
                if modoSelect < 3:
                    modoSelect=modoSelect+1
                else:
                    modoSelect=0  
            elif letra == b's' or letra == b'S':
                print("seleccionaste s")
                if modoSelect > 0:
                    modoSelect=modoSelect-1
                else:
                    modoSelect=3
            elif letra == b'd' or letra == b'D' :
                print("seleccionaste D")
                a=0
                variable=ModTt(modoSelect,a, X, Y, Z, pitchY )
                X=variable[0]
                Y=variable[1]
                Z=variable[2]
                pitchY=variable[3]
                TTTTT=generarTt(X, Y, Z,pitchY)
                q=invkinPxC(TTTTT)
                movArt(q)
            elif letra == b'a' or letra == b'A' :
                print("seleccionaste A")
                a=1
                variable=ModTt(modoSelect,a, X, Y, Z, pitchY )
                X=variable[0]
                Y=variable[1]
                Z=variable[2]
                pitchY=variable[3]
                TTTTT=generarTt(X, Y, Z,pitchY)
                q=invkinPxC(TTTTT)
                movArt(q)
            print("estoy en: ", modoSelect)
            
    except rospy.ROSInterruptException:
        pass