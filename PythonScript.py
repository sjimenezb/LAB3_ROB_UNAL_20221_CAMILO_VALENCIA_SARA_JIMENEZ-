import termios, sys, os
import rospy
from geometry_msgs.msg import Twist

from turtlesim.srv import TeleportAbsolute, TeleportRelative
from numpy import pi

import sys #para uar la linea de comandos
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




#rospy.init_node('miNodoSeLlamaAsi', anonymous=True)#creo el nodo para este programa de python
#ahora las funciones del nodo un publisher y un servicio proxy
#servi=rospy.ServiceProxy('/turtle1/teleport_absolute',TeleportAbsolute)#creo el proxy para el servicio el tipo es TeleportAbsolute lo consegui con:rosservice type /turtle1/teleport_relative 
#pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)#publicador con nombre /turtle1/cmd_vel con mensaje tipo twist y una rata de 10
#servi2=rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)

##Defino cómo variables globales las posiciones iniciales
X = -132
Y = 0
Z = 70
#anguloY=0 #aqui se la pose en grados del eje Y
pitchY = pi#deg2rad(anguloY) 
modoSelect = 0

q=[0,0,0,0]


#Crea el servicio dentro del nodo que creo en el controller.launch
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


#Funcion para detectar la tecla regresa la tecla presionada 
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

    print("este es: ", q1)

    pasitos=[math.floor((q1[0]+180)*(4096/360))]
    pasos=(pasitos)
    #int(np.sum(q1,[180.0, 180.0, 270.0, 180.0], axis=0 ))#* (4096/360))
    print(pasos)

    jointCommand('', 1, 'Goal_Position', pasos[0], 0.05)
    #jointCommand('', 2, 'Goal_Position', pasos[1], 0.05)
    #jointCommand('', 3, 'Goal_Position', pasos[2], 0.05)
    #jointCommand('', 4, 'Goal_Position', pasos[3], 0.05)
    #print("movimiento exitoso")

def ModTt(modoSelect1,tecla2, X1, Y1, Z1, pitchY1):
    print("tu pose actual es: ", X1, Y1 , Z1 , pitchY1)
    if modoSelect1 == 0 and tecla2 == 0: ##cuando a=0 es poque 
        print("estás modificando en trax + ")
        X1=X1+2
        
    elif modoSelect1 == 0 and tecla2 == 1:
        print("estás modificando en trax - ")
        X1=X1-2
    elif modoSelect1 == 1 and tecla2 == 0:
        print("estás modificando en tray + ")
        Y1=Y1+2
    elif modoSelect1 == 1 and tecla2 == 1:
        print("estás modificando en tray - ")
        Y1=Y1-2
    elif modoSelect1 == 2 and tecla2 == 0:
        print("estás modificando en traz + ")
        Z1=Z1+2
    elif modoSelect1 == 2 and tecla2 == 1:
        print("estás modificando en traz - ")
        Z1=Z1-2
    elif modoSelect1 == 3 and tecla2 == 0:
        print("estás modificando en PitchY + ")
        pitchY1=pitchY1+0.2
    elif modoSelect1 == 3 and tecla2 == 1:
        print("estás modificando en PitchY - ")
        pitchY1=pitchY1-0.2
    
    print("la nueva pose es: ", X1, Y1 , Z1 , pitchY1)
    #generarTt(X1, Y1 , Z1 , pitchY1)#Poner aqui la función de generar el Tt
    return X1, Y1 , Z1 , pitchY1

def generarTt(X, Y , Z , pitchY):
    Tt = transl(X,Y,Z)@ troty(pitchY)@ trotz(math.atan2(Y,-X))
    print(Tt)
    return(Tt)
    #invkinPxC(Tt)

def invkinPxC(Tttemp):
    ##poner aqui la función para sacar q1 q2 q3 q4 
    print("llegue hasta generar los q")


    #function q_inv = invKinPxC(varargin)
    aa=math.atan(100/32)*pi/180
    off=[pi,aa,(pi/2)-aa,0]
    T=Tttemp
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
    return  q_invu








if __name__ == '__main__':
    try:
        #artSelec=1
        
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
                #print("Estás en: ", MPos[0][artSelec-1])  
            elif letra == b's' or letra == b'S':
                print("seleccionaste s")
                if modoSelect > 0:
                    modoSelect=modoSelect-1
                else:
                    modoSelect=3
                #print("Estás en: ", MPos[0][artSelec-1])
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