# ROS2 SCARA

Paquetes para controlar un robot SCARA Epson que tenemos en la universidad usando ROS 2

## Para usar los paquetes

1. Cambiamos los permisos de los puertos

    ```bash
    sudo chmod 777 /dev/ttyACM0
    ```

2. Creamos una conexión entre el puerto real y uno virtual	

    ```bash
    socat -d -d pty,rawer,echo=0,link=/tmp/servo  /dev/ttyACM0,b115200,raw
    ```	

3. En otro terminal, lanzamos el sistema
	
    ```bash
	ros2 launch servo_hardware_moveit_config demo.launch.py
    ```	