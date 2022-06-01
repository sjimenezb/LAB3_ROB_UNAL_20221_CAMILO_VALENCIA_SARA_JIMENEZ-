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


#rospy.init_node('miNodoSeLlamaAsi', anonymous=True)#creo el nodo para este programa de python
#ahora las funciones del nodo un publisher y un servicio proxy
#servi=rospy.ServiceProxy('/turtle1/teleport_absolute',TeleportAbsolute)#creo el proxy para el servicio el tipo es TeleportAbsolute lo consegui con:rosservice type /turtle1/teleport_relative 
#pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)#publicador con nombre /turtle1/cmd_vel con mensaje tipo twist y una rata de 10
#servi2=rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)

##Defino cómo variables globales las posiciones iniciales
X = -32
Y = 0
Z = 347
anguloY=0 #aqui se la pose en grados del eje Y
pitchY = 0#deg2rad(anguloY) 
modoSelect = 0

q1=0
q2=0
q3=0
q4=0


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

def movArt(q1, q2, q3, q4):
    jointCommand('', numArt, 'Goal_Position', pos, 0.5)

def ModTt(modoSelect1,tecla2, X1, Y1, Z1, pitchY1):
    print("tu pose actual es: ", X1, Y1 , Z1 , pitchY1)
    if modoSelect1 == 0 and tecla2 == 0:
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
    invkinPxC(Tt)

def invkinPxC(Tttemp):
    ##poner aqui la función para sacar q1 q2 q3 q4 
    print("llegue hasta generar los q")

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
                a=1
                variable=ModTt(modoSelect,a, X, Y, Z, pitchY )
                X=variable[0]
                Y=variable[1]
                Z=variable[2]
                pitchY=variable[3]
                generarTt(X, Y, Z,pitchY)
            elif letra == b'a' or letra == b'A' :
                print("seleccionaste A")
                a=0
                variable=ModTt(modoSelect,a, X, Y, Z, pitchY )
                X=variable[0]
                Y=variable[1]
                Z=variable[2]
                pitchY=variable[3]
                generarTt(X, Y, Z,pitchY)
            print("estoy en: ", modoSelect)
            
    except rospy.ROSInterruptException:
        pass