Date of creation 27/May/2022.

This repository will contain the work from Camilo Valencia and Sara Jiménez for the course "Robótica 2022-1" in the Universidad Nacional de Colombia Sede Bogotá, continuing with "Taller 3"

# Laboratorio 3: Cinemática Inversa - Robot Phantom X - ROS

En este repositorio se pueden encontrar los códigos para el desarrollo del laboratorio 3 de la clase de robótica
## Sección 1: Modelo cinemático inverso

#### Materiales
- Modelo Robot PhantomX Pincher

### Obtención del modelo mediante método de desacopple de muñeca y geométrico

 
![Modelo](https://user-images.githubusercontent.com/55710287/168397177-2eed1805-2caf-475f-b450-df4d8117a0ef.png)


### Diagama y parámetros DH


| j  | a<sub>i</sub> | α<sub>i</sub>  | d<sub>i</sub> | θ<sub>i</sub>| Offset |
|----|---------------|----------------|---------------|--------------|--------|
| 1  | 0             | -π/2           | 47            | q<sub>1</sub>|   π    |
| 2  | 105.95        |    0           | 0             | q<sub>2</sub>| 1.5986π|
| 3  | 100           |    0           | 0             | q<sub>3</sub>| 0.4014π|
| 4  | 100           |    0           | 0             | q<sub>4</sub>|   0    |


Sin embargo para nuestro análisis decidimos trabajar con una postura de Home distinta, cuyo modelo DH y tabla de parámetros son los siguientes:

![DHHome](https://user-images.githubusercontent.com/55710287/168397356-89bdb070-9af9-451f-a3cb-817231a646c1.png)

| j  | a<sub>i</sub> | α<sub>i</sub>  | d<sub>i</sub> | θ<sub>i</sub>| Offset |
|----|---------------|----------------|---------------|--------------|--------|
| 1  | 0             | -π/2           | 47            | q<sub>1</sub>|   π    |
| 2  | 105.95        |    0           | 0             | q<sub>2</sub>|-0.4014π|
| 3  | 100           |    0           | 0             | q<sub>3</sub>|-0.0986π|
| 4  | 100           |    0           | 0             | q<sub>4</sub>|   0    |
    



## Sección 2: Matlab + Toolbox
#### Materiales:
Los materiales para esta sección del trabajo son:
- Robot PhantomX Pincher
    - 6 motores Dynamixel AX12
    - Fuente 12V
    - FTDI
    - HUB
- Computador
    - Ubuntu 20.04
    - Ros noetic
    - Dynamixel 



### Metodología y Resultados

Explicación del programa



### Análisis:

Como se puede observar 

## Sección 3: ROS - Pick and Place
### Materiales
- Robot PhantomX Pincher
    - 6 motores Dynamixel AX12
    - Fuente 12V
    - FTDI
    - HUB
- Computador
    - Ubuntu 20.04
    - Matlab R2020b 
### Metodología y Resultados


### Análisis:


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
    - Dynamixel
### Metodología y Resultados

### Análisis:



## Conclusiones:
- El terminal es una herramienta extremadamente útil para manejar el sistema de archivos, además de tener ciertas capacidades que no son posibles de alcanzar mediante las alternativas con GUI.
- Matlab a su vez tiene una gran capacidad para automatizar procesos en ROS como la recolección de datos, control de motores, recibir información del robot y graficarlo, al igual que realizar cálculos complejos, algoritmos de evasión de objetos y mucho más.
- Es posible obtener en Matlab toda la información que proporciona ROS en el terminal, de una manera más organizada al poder emplear todas las capcidades del software destinadas al manejo de datos.
- Al manejar nodos en Matlab se debe tener especial cuidado, pues no es posible correr más de una instancia del mismo, al igual que las relaciones de publicador/suscriptor entre ellos.
- Herramientas cómo Dynamixel wizard permiten controlar motores desde una interfaz gráfica que proporciona la capacidad de prender y apagar los torques al igual que configurar los ángulos del motor evidenciando los pasos requeridos, siendo una herramienta fundamental al momento de recolección de información sobre cada motor en una pose.
- Python al tener acceso a las librerías de toda la web permite usarlas al momento de controlar robots por medio de ros, aumentando las capacidades de un script. 
- Los servicios pueden resultar de gran utilidad para poder manipular parámetros que no serían accesibles de forma directa de otras maneras.
- Python es una excelente herramienta para manejar la interfaz humano máquina, además de todas las ventajas que nos ofrece este lenguaje como librerías, estructuras de datos, etc.
- Además permite ejectuar complejas rutinas de manera sencila en un solo comando del terminal con ROS configurado.