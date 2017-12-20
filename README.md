# Proyecto Finla de Robotoca Temas Selectos de Robotica

### Compilacion

En este repositorio se encuntra la carpeta *control* que es el workspace del proyecto del contról cinemático, para compilar se debe se ejecutar *catkin_make* dentró de esta carpeta (puede estar en ubucación deseada del equipo con el que se compile). Adicionalmente se debe contar con el modelo del carro autonomo y Gazebo instalados en la máquina. En este vínculo https://github.com/EagleKnights/SDI-11911/wiki se encuntra el módelo requerido y las instrucciones para instalar Gazebo.

### Estructura del paquete

La estructura del paquete que se encuntra en este repositorio es la siguiente:
* control (workspace del paquete ROS)
    * src (Ubicación de los archivos del paquete)
        * control (Ubicación del código fuente, el archivo Cmake y el manifiesto)

### Ejecución del paquete

Después de compilar y agregar las variebles de ambiente se obtendrá un paquete de ROS llamado **control** con un ejecutable:
*   *pose_controller* Es el programa que contiene el control de carro. No recibe parámetros de entrada, por el momento las metas del control están definidas dentro del código y se pueden modificar en la función main. 

    rosrun control pose_controller