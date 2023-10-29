# 4. Primeros pasos con ROS

## 4.1. Creando un paquete en ROS

Todo el desarrollo del proyecto será realizado en la plataforma _ROS_, en el caso de nunca haber trabajado con ROS, se recomienda seguir los tutoriales disponibles en la [Documentación](https://docs.ros.org/en/humble/Tutorials.html) o ver el siguiente [video](https://www.youtube.com/watch?v=Gg25GfA456o&t=2336s) para obtener conceptos básicos de esta herramienta. 

Cuando se trabaja en ROS, básicamente se tiene una carpeta global llamada _Workspace_ en la que se encuentran varias carpetas pero la más importante es una llamada _src_, allí se encuentran a su vez más carpetas conocidas como _packages_ o paquetes, estos paquetes contienen todos los scripts necesarios para realizar una tarea. Por ejemplo, podemos tener un package para solamente visualizar el robot en _rviz_, o para moverlo en _gazebo_; es decir, el número de paquetes depende directamente del desarrollador. 

Para crear un paquete se puede utilizar un propio comando de ubuntu `ros2 pkg create my_first_package ...` estos paquetes pueden crearse ya sea en _Python_ o en _C++_, esto dependerá del desarrollador; además, se deben especificar las dependecias del paquete para no ocasionar ningún error; sin embargo, en el caso de no especificarlas completamente, se pueden añadir después sin ningún problema. En este orden de ideas para comenzar con la creacion de todo se seguirían los siguientes pasos:

1. Crear Workspace `mkdir robot_ws` 
2. Crear carpeta src `cd robot_ws`  `mkdir src`  `cd src` 
3. Crear paquete `ros2 pkg create my_first_package --build-type ament_python --dependencies rclpy`

Con esto ya tendrías el paquete creado, sería un paquete vacío hasta el momento. Lo siguiente sería compilar el paquete, al compilarlo se crearán varias carpetas en el workspace: _log_, _install_ y _build_, para compilar basta con escribir el comando `colcon build`, en el caso de no tenerlo instalado simplemente sigue el comando que te recomeinda la terminal de ubuntu. Cabe resaltar que todo esto puedes hacerlo desde la terminal nativa de WSL o para mas facilidad desde _Visual Studio Code_ al abrir una de tipo _Ubuntu(WSL)_.


## 4.2. Clonar un paquete de Github

A pesar de que el proceso anterior no sea tan tedioso, otra opción para crear un paquete puede ser utilizando una plantilla de Github y adaptarla a tus necesidades. Para esto basta con seguir los siguientes pasos: 

1. Crear Workspace `mkdir robot_ws` 
2. Crear carpeta src `cd robot_ws` `mkdir src` `cd src` 
3. Clonar `git clone https://github.com/Ph0n1x0/my_bot.git`, esta plantilla es la utilizada en el [canal de referencia](https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) previamente mencionado.

Con esto ya tendrías una plantilla lista para modificar. 


Es importante mencionar que cada que cambies script del paquete debes usar el comando `colcon build` y `source install/setup.bash`

**TIP:** Si utilizas el comando `colcon build  --symlink-install` compilará automaticamente los cambios en los scripts, solamente _NO_ compilará cuando crees nuevos documentos o archivos.
