# 7. Scara Xacros 

<p style="text-align: justify;">
En el repositorio previamente expuesto se encuentran distintos archivos xacro. A continuación, se enlistan los archivos con una pequeña descripción de los mismos. Además, se mencionan las principales características de cada uno:
</p style="text-align: justify;">

1. **Gazebo:** <p style="text-align: justify;"> Descripción de robot en materiales (colores) y en hardware component con toda la integración de ros2 control, en esta parte se describe el plugin necesario y los joints del robot con sus `command interfaces` y `state interfaces`. </p style="text-align: justify;">
2. **Ros2 control:** <p style="text-align: justify;">Descripción de robot para aplicación en Scara _REAL_, para esto se utiliza como plugin los archivos de scara Hardware y parámetros importantes para el control como el `Baud rate`, constantes `PID`, `Serial Device`, entre otros.</p style="text-align: justify;"> 
3. **URDF:** <p style="text-align: justify;"> URDF extraído de Solidworks según el ejemplo visualizado, este URDF usa _meshes_ para parte del robot definida:`antebrazo`, `base`, `brazo`,`dedo` y `mano`.  </p style="text-align: justify;">


## 7.1. Gazebo xacro 


<p style="text-align: justify;"> 

Un concepto clave para los archivos xacros que se debe tener en claro es la diferenciación entre `command interfaces` y `state interfaces`. Las `command interfaces` se utilizan como recurso para enviar _comandos_ deseados para controlar el robot/dispositivo; mientras que, las `state interfaces`se usan para saber los _estados_ actuales de parámetros del robot. Por ejemplo en el scara se tiene como  `command interface` la posición del servo, porque se desea mover el servo ingresándole _comandos_ de posición; mientras que, para `state interfaces` se tiene la posición y velocidad del scara. La selección de estos parámetros dependerá siempre del controlador </p style="text-align: justify;"> 


[Enlace del controlador](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html)



Hablando un poco del archivo xacro, se tiene al inicio donde se incluye el URDF del Scara y su respectivo archivo de `ros2_control` 

```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="epson">

    <!-- Import scara urdf file -->
    <xacro:include filename="$(find scara)/urdf/urdf.xacro" />

    <!-- Import brazo ros2_control description -->
    <xacro:include filename="$(find scara_hardware)/config/ros2_control.xacro" />
    <xacro:hardware_component 
        name="GazeboSimSystem" 
        plugin="gazebo_ros2_control/GazeboSystem" 
    />
```
<p style="text-align: justify;">

Después de esto se tiene la definición de colores para cada parte del robot definida
</p style="text-align: justify;">

```xml 
    <!-- Gazebo Colors  -->
    <gazebo reference="base_link">
        <material>Gazebo/Orange</material>
    </gazebo>
    <gazebo reference="antebrazo">
        <material>Gazebo/White</material>
    </gazebo>
    <gazebo reference="brazo">
        <material>Gazebo/White</material>
    </gazebo>
    <gazebo reference="mano">
        <material>Gazebo/White</material>
    </gazebo>
    <gazebo reference="dedo">
        <material>Gazebo/White</material>
    </gazebo>
```

Por último, se tienen unas lineas en donde se ingresa la ruta para encontrar el controlador del robot, este archivo es un script _.YAML_ y se describirá más adelante.
```xml 
    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <parameters>$(find scara)/config/ros2_controllers.yaml</parameters>
        </plugin>
    </gazebo>

</robot>
```

## 7.2. ROS2 Control 

El xacro del _ros2 Control_ se utiliza cuando se desea controlar el scara _REAL_ conectado al PC, para esto se necesiten los archivos en _C++_ que se describirán más adelante. Además de eso, se ingresan parámetros necesarios para el control del dispoitivo. A diferencia del archivo del servo, el archivo del Scara es más _robusto_ porque se necesitan especificar parámetros para cada joint como `joint_controller_gain` , `joint_offset`, `joint3_minimum_time_period`, etc.   
```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="hardware_component" params="name plugin">
    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>${plugin}</plugin>

        <!-- Joint1 -->
        <param name="joint1_name">hombro</param>
        <param name="joint1_controller_gain">50.0</param>
        <param name="joint1_counts_per_unit">133005.464</param>
        <param name="joint1_ref">-1.84</param>
        <param name="joint1_minimum_time_period">1</param>
        <param name="joint1_maximum_time_period">1000000</param>
        <param name="joint1_offset">243400</param>

        <!-- Joint2 -->
        <param name="joint2_name">codo</param>
        <param name="joint2_controller_gain">50.0</param>
        <param name="joint2_counts_per_unit">15481.74</param>
        <param name="joint2_ref">2.57</param>
        <param name="joint2_minimum_time_period">3</param>
        <param name="joint2_maximum_time_period">1000000</param>
        <param name="joint2_offset">-39850</param>

        <!-- Joint3 -->
        <param name="joint3_name">muneca</param>
        <param name="joint3_controller_gain">50.0</param>
        <param name="joint3_counts_per_unit">1000.0</param>
        <param name="joint3_minimum_time_period">1</param>
        <param name="joint3_maximum_time_period">1000000</param>
        <param name="joint3_offset">0</param>

        <!-- Joint4 -->
        <param name="joint4_name">falange</param>
        <param name="joint4_controller_gain">50.0</param>
        <param name="joint4_counts_per_unit">1065200.0</param> <!-- 2 * 79890 / 0.15m -->
        <param name="joint4_minimum_time_period">1</param>
        <param name="joint4_maximum_time_period">1000000</param>
        <param name="joint4_offset">0</param>
```

Luego de esto se definen los parámetros para la `serial_communication` y las `command interfaces` y `state interfaces`

```xml 
 <!-- Serial Communication -->
        <param name="loop_rate">30</param>
        <param name="device">/tmp/scara</param>
        <param name="baud_rate">115200</param>
        <param name="timeout_ms">1000</param>
        <param name="echo">true</param>
      </hardware>
      <joint name="hombro">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">-1.84</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
      <joint name="codo">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">2.57</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
      <joint name="falange">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
      <joint name="muneca">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
    </ros2_control>
  </xacro:macro>
</robot>

```
## 7.3. Calibración para ROS2 Control 

Explicar proceso de calibración 