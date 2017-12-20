# Proyecto Finla de Robotoca Temas Selectos de Robotica

### Compilacion

En este repositorio se encuntran dos carpetas correspondientes a dos nodos en ROS: *ProyectoFinal_Parte1* que es el workspace del proyecto del correspondientes a la primera pregunta del examen, y *ProyectoFinal_Parte1&2* que es el workspace de las preguntas 2 y 3. Para compilar ambos nodos se debe se ejecutar *catkin_make* dentró de ambas carpets (puede estar en ubucación deseada del equipo con el que se compile). 

Para la parte 1, adicionalmente se debe de contar con una bolsa de ROS ubicada en: robotica.itam.mx/rosbags/rosbag_SDI11911.bag 

Para la parte 2 y 3, adicionalmente se debe contar con el modelo del carro autonomo y Gazebo instalados en la máquina. En este vínculo https://github.com/EagleKnights/SDI-11911/wiki se encuntra el módelo requerido y las instrucciones para instalar Gazebo. Este proyecto tambien contiene la libreria Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page) para el manejo de matrices.

### Estructura del paquete

La estructura del paquete que se encuntra en este repositorio es la siguiente:
* ProyectoFinal_Parte1 (workspace del paquete ROS)
    * src (Ubicación de los archivos del paquete)
        * proyecto2 (Ubicación del código fuente, el archivo Cmake y el manifiesto)

* ProyectoFinal_Parte2&3 (workspace del paquete ROS)
    * src (Ubicación de los archivos del paquete)
        * final (Ubicación del código fuente, el archivo Cmake y el manifiesto)

### Ejecución del paquete

Después de compilar y agregar las variebles de ambiente se obtendrán dos paquetes de ROS llamado **proyecto31** y **final**, con un ejecutable cada uno:
*   *proyecto2* Es el programa que hace el análisis de la ubicación de la posición del vahículo a partir de su posición en la carretera.

    rosrun proyecto2 proyecto31

*   *final* Es el programa que a partir de las lecturas del LiDar observa un obstaculo y hace una estamación de su estado a partir de un filtro de kalman. También pública las variables de control del vehículo al modelo de Gazebo.

    rosrun final final