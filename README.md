Date of creation 27/May/2022.

This repository will contain the work from Camilo Valencia and Sara Jiménez for the course "Robótica 2022-1" in the Universidad Nacional de Colombia Sede Bogotá, continuing with "Taller 3"

# Laboratorio 3: Cinemática Inversa - Robot Phantom X - ROS

En este repositorio se pueden encontrar los códigos para el desarrollo del laboratorio 3 de la clase de robótica
## Sección 1: Modelo cinemático inverso

#### Materiales
- Modelo Robot PhantomX Pincher

### Obtención del modelo mediante método de desacopple de muñeca y geométrico

 Para obtener las ecuaciones necesarias en el control del robot mediante cinemática inversa se hizo uso del método de desacople de muñeca junto al análisis geométrico de un mecanismo tipo 2R que se obtenía a partir de las articulaciones 2, 3 y 4, tal como se vió en las clases pasadas con el fin de llegar a una expresión para θ1, θ2 y θ3; siendo θ4 obtenido mediante un análisis de rotación de las matrices herramienta y base

![2R](https://user-images.githubusercontent.com/55710287/171786631-419aff33-58b0-4289-8da8-1d080df094b4.png)

## Sección 2: Matlab + Toolbox
#### Materiales:
Los materiales para esta sección del trabajo son:
- Computador
    - Ubuntu 20.04
    - Matlab R2020b con mensajes Dynamixel 
    - Toolbox de robótica de Peter Corke V9.10

### Espacio de trabajo
El espacio de trabajo fue esbozado mediante un sencillo script de Matlab graficando miles de puntos alcanzables por el robot.

![Workspace](https://user-images.githubusercontent.com/55710287/171786687-b3461d14-9190-4925-ac85-d227ab7cfc4d.png)

### Métodos para hallar la cinemática inversa en el ToolBox
En el toolbox sde Peter Corke hay un total de 5 funciones asociadas al objeto SerialLink para obtener la cineática inversa de un robot, cada una con sus particularidades:
- .ikine6s: cinemática inversa para un robot de 6 DOF con mmuñeca esférica
- .ikine:  cinemática inversa mediante métodos numéricos
- .ikunc: cinemática inversa mediante toolbox de optimización
- .ikcon: cinemática inversa mediante toolbox de optimización con límites de articulaciones
- .ikine: cinemática inversa analítica obtenida mediante elementos sybólicos
### Preguntas de análisis
- En el caso del Phantom X, el GDL restante es el cuarto ángulo, y este corresponde a una orientación de pitch, es decir, una orientación alrededor del eje Y o Open del efector final.
- Como se vió en las ecuaciones, dado que el seno de θ3 corresponde a una raíz, esto implica que existen 2 posibles soluciones: una configuración llamada codo arriba y otra codo abajo.
- El espacio diestro de un robot es aquel espacio donde puede llegar a cualquier posición con una orientación arbitraria, es decir, con múltiples oprientaciones/ángulos llega al mismo punto del espacio cartesiano

## Sección 3: ROS - Aplicación Pick and Place:
### Materiales
Los materiales para esta sección del trabajo son:
- Robot PhantomX Pincher
    - 6 motores Dynamixel AX12
    - Fuente 12V
    - FTDI
    - HUB
- Computador
    - Ubuntu 20.04
    - Matlab R2020b con mensajes Dynamixel 
    - Toolbox de robótica de Peter Corke V9.10 
### Metodología y Resultados
Para esta sección del taller, se busca mediante un script de Matlab generar una serie de trayectorias que el robot PhantomX siga y con ello realice una secuencia pick and place de aros en un objetivo.
Como base tomamos el script compartido en el repositorio de la clase 6 el cual será nuestro script principal ( `PXinvKine.m` ). Este lo modificamos para adaptarse al modelo del phantom nuevo desarrollado en laboratorios anteriores, implementamos nuestras ecuaciones de cinemática inversa y definimos todas las poses necesarias para el diseño de las trayectorias.

![invKinTest](https://user-images.githubusercontent.com/55710287/171786702-8761793d-a7f5-45b9-97a5-0e11d9549135.png)

En la sección de código mostrada se ve como deifnimos una pose concreta a partir de unas coordenadas cartesianas y una orientación como giro alrededor del eje Y u Open en notación NOA de la herramienta. Una vez definida se realiza el cálculo correspondiente de los ángulos θ1 a θ4 en las dos configuraciones posibles y se grafica una de ellas:

![invKinTestRobot](https://user-images.githubusercontent.com/55710287/171786715-43464e96-f8dc-48ff-a60b-40f9a1a7909d.png)

Tras comprobar que el modelo que se comporta de manera adecuada procedemos a definir todas las posturas que necesitamos en la secuencia pick and place:

![Poses](https://user-images.githubusercontent.com/55710287/171786732-669c48d3-5d2d-447b-8671-142fddbe7d5b.png)

Y sus trayectorias mediante la función `ctraj()` del Toolbox:

![Trayectorias](https://user-images.githubusercontent.com/55710287/171786754-72874c53-e494-45c5-90e5-43e1477130c2.png)

Finalmente se confirma que la simulación coincida con la trayectoria deseada:

![simulacion](https://user-images.githubusercontent.com/55710287/171786768-fbba5c5a-418c-4fa7-89f5-381ed8dd285e.gif)

Y se procede a controlar el PhantomX para que complete la aplicación como se ve en el siguiente video empleando las funciones desarroladas en laboratorios anteriores: https://youtu.be/04ysbdnuB_4

(Click en la imagen para acceder directamente)

[![pick and place](https://img.youtube.com/vi/04ysbdnuB_4/mqdefault.jpg)](https://youtu.be/04ysbdnuB_4) 

### Análisis:
Como se puede observar, el robot Phantom X sigue de forma muy cercana la trayectoria simulada, sin embargo debido a discrepancias en la función `ctraj()` a la hora de calcular ciertos pasos y con ellos los ángulos que resultan en las ecuaciones, no se ve un comportamiento idéntico en el video, sin embargo sí nos permite dar una excelente aproximación del comportamiento del robot real siempre y cuando el modelo empleado sea mediananmente preciso. La principal dificultad fue obtener unas poses las cuales tuviesen unas trayectorias alcanzables por el robot real, pues este cuenta con límites de articulaciones no contempladas de forma correcta en MATLAB a pesar de que se especifican, y colisiones consigo mismo, así como lograr unas trayectorias aceptablemente precisas.
Adicionalmente se puede observar un movimiento con muchas vibraciones. Esto es debido a que, a pesar de que se definen varias poses interpoladas entre cada punto de ruta, el movimiento entre cada una lo realiza de manera repentina, sin control de torque o velocidad.
## Sección 4: ROS - Aplicación de movimiento en el espacio de tarea

### Materiales
- Robot PhantomX Pincher
    - 6 motores Dynamixel AX12
    - Fuente 12V
    - FTDI
    - HUB
- Computador
    - Ubuntu 20.04
    - Matlab R2020b 
    - Dynamixel Wizard
### Metodología y Resultados
Para finalizar el laboratorio, realizamos el control del robot PhantomX a través de Python empleando mensajes de Dynamixel, funciones de RO y varias librerías para Python incluyendo la de Peter Corke.
El script de Python mostrado a continuación `PythonScript.py` contiene todo lo necesario para este punto, además de asegurarse de correr `px_controllers.launch`  para las respectivas comunicaciones con el robot.

Explicación del programa:

Para realizar este programa, iniciamos definiendo los incrementos que se van a realizar según las teclas presionadas, para luego realizar el programa de python que por medio de cinemática inversa según las posiciónes de la herramienta que el usuario indique se calculen las posiciones de cada motor.

Este script tiene el trabajo de navegar entre los tipos de movimiento usando  "w" y "s" y modificando la posición final del efector  "a" y "s". Para esto realizamos las siguientes funciones:
- ```jointCommand```: Esta función permite crear el servici'deentro del nodo del launch para comunicarse con el robot.
- ```getkey```: Esta función permite detectar la tecla que ha sido presionada en el teclado y la retorna.
- ```mov Art```: Esta función recibe un arreglo con los valores en radianes con las posiciones deseadas de cada motor y calcula los pasos que se requieren para la posición que deseamos.
- ```ModTt```: Esta función permite tomar los valores actuales de la matriz de transformación Tt y según la tecla presionada por el usuario incrementa o disminuye los movimientos de la herramienta.
- ```generar Tt```: Esta función es la que  recibe los diferentes valores de posición de la herramienta y con esto genera la matriz Tt.
- ```invkinPxC```: Esta función recibe la Tt generada por ```generar Tt``` y por medio de cinemática inversa retorna un arreglo con los valores de q para cada una de las articulaciónes, esta función es la misma que usamos en el script de Matlab.
    
Finalmente pasamos al main que recibe la información de las teclas presionadas usando ```getkey``` luego con esta información según la tecla presionada se cambia de tipo de movimiento  ```modoSelect``` con "w" o "s" donde si se llega al rot se presiona s se vuelve al trax y viceversa. Todo esto se encuentra en un loop.
     
![main](https://user-images.githubusercontent.com/82957735/171980544-375216e1-4a07-4e78-8989-f3e17541537f.jpg)

Para finalizar se pueden observar los resultados de nuestro control mediante  script de python, en el siguiente video: 
https://youtu.be/ncu0eVfDIHw

### Análisis:
Cómo se puede observar en el vídeo, Python nos permite generar scripts, lo cual es muy útil a la hora de comunicar por medio de un servicio en ros, además el amplo número de librerías de python permite detectar facilmente las teclas presionadas, realizar operaciones matemáticas y emplear el toolbox de Peter Corke. Respecto a los resultados obtenidos podemos decir que son satisfactorios: Se logró controlar el PhantomX mediante el manejo de comandos dynamixel exitosamente, además por medio de las diferentes funciones se puede navegar entre los tipos de movimiento sin problema para así en cada una variar su posición.



## Conclusiones:
- El terminal es una herramienta extremadamente útil para manejar el sistema de archivos, además de tener ciertas capacidades que no son posibles de alcanzar mediante las alternativas con GUI.
- Matlab a su vez tiene una gran capacidad para automatizar procesos en ROS como la recolección de datos, control de motores, recibir información del robot y graficarlo, al igual que realizar cálculos complejos, algoritmos de evasión de objetos y mucho más.
- Es posible obtener en Matlab toda la información que proporciona ROS en el terminal, de una manera más organizada al poder emplear todas las capcidades del software destinadas al manejo de datos.
- Al manejar nodos en Matlab se debe tener especial cuidado, pues no es posible correr más de una instancia del mismo, al igual que las relaciones de publicador/suscriptor entre ellos.
- Herramientas cómo Dynamixel wizard permiten controlar motores desde una interfaz gráfica que proporciona la capacidad de prender y apagar los torques al igual que configurar los ángulos del motor evidenciando los pasos requeridos, siendo una herramienta fundamental al momento de recolección de información sobre cada motor en una pose.
- Python al tener acceso a las librerías de toda la web permite usarlas al momento de controlar robots por medio de ros, aumentando las capacidades de un script. 
- Un factor muy importante al realizar la sección de Python fue poder emplear funciones del Tool box de Peter Corke que permite facilmente sacar diferentes matrices de rotación, entre otras funciones.
- Los servicios pueden resultar de gran utilidad para poder manipular parámetros que no serían accesibles de forma directa de otras maneras.
- Python es una excelente herramienta para manejar la interfaz humano máquina, además de todas las ventajas que nos ofrece este lenguaje como librerías, estructuras de datos, etc.
- Además permite ejectuar complejas rutinas de manera sencila en un solo comando del terminal con ROS configurado.
