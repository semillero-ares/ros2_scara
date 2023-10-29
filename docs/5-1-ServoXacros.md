# 7. Servo Xacros 

<p style="text-align: justify;">
En el repositorio previamente expuesto se encuentran distintos archivos xacro. A continuación, se enlistan los archivos con una pequeña descripción de los mismos. Además, se mencionan las principales características de cada uno:
</p style="text-align: justify;">

1. **Gazebo:** <p style="text-align: justify;"> Descripción de robot en materiales (colores) y en hardware component con toda la integración de ros2 control, en esta parte se describe el plugin necesario y los joints del robot con sus `command interfaces` y `state interfaces`. </p style="text-align: justify;">
2. **Ros2 control:** <p style="text-align: justify;">Descripción de robot para aplicación en servo _REAL_, para esto se utiliza como plugin los archivos de servo Hardware y parámetros importantes para el control como el `Baud rate`, constantes `PID`, `Serial Device`, entre otros.</p style="text-align: justify;"> 
3. **URDF:** <p style="text-align: justify;">Descripción general del robot, al haber sido realizada con geometrías básicas también se encuentran archivos como `Inertial macros` y `Servo materials`; además, al ser un repositorio que cuenta con la opción de simular el servo en _Gazebo_ y luego realizar la aplcación con el servo real, se encuentran dos archivos para cada una de estas: `servo.simulated.xacro` y `servo.urdf.xacro` respectivamente. </p style="text-align: justify;">


## 7.1. Gazebo xacro 


<p style="text-align: justify;"> 

Un concepto clave para los archivos xacros que se debe tener en claro es la diferenciación entre `command interfaces` y `state interfaces`. Las `command interfaces` se utilizan como recurso para enviar _comandos_ deseados para controlar el robot/dispositivo; mientras que, las `state interfaces`se usan para saber los _estados_ actuales de parámetros del robot. Por ejemplo en el servo se tiene como  `command interface` la posición del servo, porque se desea mover el servo ingresándole _comandos_ de posición; mientras que, para `state interfaces` se tiene la posición y velocidad del servo. La selección de estos parámetros dependerá siempre del controlador </p style="text-align: justify;"> 
<center>

[Enlace del controlador](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html)
</center>


Hablando un poco del archivo xacro, se tiene al inicio la descripción de los colores del robot 

```xml 
<?xml version="1.0"?>
<robot>

    <!-- Gazebo Colors -->
    <gazebo reference="motor">
        <material>Gazebo/Blue</material>
    </gazebo>
    <gazebo reference="arm">
        <material>Gazebo/White</material>
    </gazebo>
```
<p style="text-align: justify;">

Después de esto se tiene la integración con el hardware component, en donde se utiliza como plugin la opción de _GazeboSystem_ al ser el xacro el servo simulado, después se describen los joints con los límites y las `command interfaces` y `state interfaces`.
</p style="text-align: justify;">

```xml 
    <!-- Gazebo Hardware Component -->

    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        <joint name="servo_joint">
            <command_interface name="position">
                <param name="min">-1.58</param>
                <param name="max">1.58</param>
            </command_interface>
            <state_interface name="position">
                <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>
```

Por último, se tienen unas lineas en donde se ingresa la ruta para encontrar el controlador del robot, este archivo es un script _.YAML_ y se describirá más adelante.
```xml 
    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <parameters>$(find servo_hardware)/config/servo_controllers_gazebo.yaml</parameters>
        </plugin>
    </gazebo>

</robot>
```

## 7.2. ROS2 Control 

El xacro del _ros2 Control_ se utiliza cuando se desea controlar el servo _REAL_ conectado al PC, para esto se necesiten los archivos en _C++_ que se describirán más adelante. Además de eso, se ingresan parámetros necesarios para el control del dispoitivo.
```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="servo_ros2_control" params="name prefix">
    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>servo_hardware/ServoArduinoHardware</plugin>
        <param name="joint1_name">servo_joint</param>
        <param name="loop_rate">30</param>
        <param name="device">/home/jammy/servo</param>
        <param name="baud_rate">115200</param>
        <param name="timeout_ms">1000</param>
        <param name="enc_counts_per_rev">3436</param>
        <param name="pid_p">20</param>
        <param name="pid_d">12</param>
        <param name="pid_i">0</param>
        <param name="pid_o">50</param>
      </hardware>
      <joint name="${prefix}servo_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
  </xacro:macro>
</robot>
```

## 7.3. URDF 

Como se menciona anteriormente, el URDF de este robot contiene distintos archivos _xacro_ como las `Inertial macros` y `Servo materials`

### 7.3.1. Inertial Macros y Servo materials 

Estos archivos son bastante similares a los vistos en el ejemplo del robot diferencial, las `Inertial macros` siempre dependerán de la complejidad geométrica del robot:
```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >

    <!-- Specify some standard inertial calculations https://en.wikipedia.org/wiki/List_of_moments_of_inertia -->
    <!-- These make use of xacro's mathematical functionality -->

    <xacro:macro name="inertial_sphere" params="mass radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(2/5) * mass * (radius*radius)}" ixy="0.0" ixz="0.0"
                    iyy="${(2/5) * mass * (radius*radius)}" iyz="0.0"
                    izz="${(2/5) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>  

    <xacro:macro name="inertial_box" params="mass x y z *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                    izz="${(1/12) * mass * (x*x+y*y)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
                    izz="${(1/2) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>
</robot>
```
<p style="text-align: justify;">
Como se puede observar, es literalmente la misma macro empleada en el robot diferencial al utilizarse las mismas figurás volumétricas. Para la parte de los materiales del robot, también se realizaron de manera equivalente como en el robot diferencial: 
</p style="text-align: justify;">
```xml 
<?xml version="1.0"?>
<!--
Copied from ROS1 example:
https://github.com/ros-simulation/gazebo_ros_demos/blob/kinetic-devel/rrbot_description/urdf/materials.xacro
-->
<robot>
  <material name="white">
    <color rgba="1 1 1 1" />
  </material>

  <material name="orange">
    <color rgba="1 0.3 0.1 1"/>
  </material>

  <material name="blue">
    <color rgba="0.2 0.2 1 1"/>
  </material>

  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
</robot>
```

### 7.3.2. Servo description 

En el xacro de servo description se tiene _TODO_ el URDF del servo empleado en el repositorio. Para esto primero se incluyen las macros previamente expuestas:
```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >

    <xacro:include filename="$(find servo_hardware)/urdf/inertial_macros.xacro" />
    <xacro:include filename="$(find servo_hardware)/urdf/servo.materials.xacro" />
```

Luego se empieza con la definición de los `links` y `joints` primero con el _MOTOR_ o _CHASIS_ del servo. 

```xml 
    <!-- BASE LINK -->
    <link name="base_link" />
    <joint name="fixation" type="fixed">
        <parent link="base_link"/>
        <child link="motor"/>
        <origin xyz="0 0 0.267"/>
    </joint>

    <!-- MOTOR LINK -->

    <link name="motor">
        <visual>
            <origin xyz="0.0 0.0535 -0.1535"/>
            <geometry>
                <box size="0.118 0.225 0.227"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <visual>
            <origin xyz="0.0 0.0 -0.025"/>
            <geometry>
                <cylinder radius="0.059" length="0.05"/>
            </geometry>
        </visual>
        <visual>
            <origin xyz="0.0 0.063 -0.025"/>
            <geometry>
                <cylinder radius="0.025" length="0.05"/>
            </geometry>
        </visual>
        <visual>
            <origin xyz="0.0 0.0535 -0.0955"/>
            <geometry>
                <box size="0.118 0.319 0.025"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz="0.0 0.0535 -0.1335"/>
            <geometry>
                <box size="0.118 0.319 0.267"/>
            </geometry>
        </collision>
        <xacro:inertial_box mass="0.15" x="0.118" y="0.319" z="0.267">
            <origin xyz="0.0 0.0535 -0.1335" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>
```

Y luego con el _ARM_ del servo o _PLUMILLA_
```xml 
    <!-- ARM LINK -->

    <joint name="servo_joint" type="revolute">
        <limit lower="-${pi/2}" upper="${pi/2}" effort="0.0" velocity="0.0"/>
        <parent link="motor"/>
        <child link="arm"/>
        <origin xyz="0 0 0.042" rpy="0 0 0" />
        <axis xyz="0 0 1"/>
    </joint>

    <link name="arm">
        <visual>
            <origin xyz="0.0 0.0 -0.027"/>
            <geometry>
                <cylinder radius="0.023" length="0.03"/>
            </geometry>
            <material name="white"/>
        </visual>
        <visual>
            <origin xyz="0.0 0.0 -0.021"/>
            <geometry>
                <cylinder radius="0.05" length="0.054"/>
            </geometry>
        </visual>
        <visual>
            <origin xyz="0.127 0.0 -0.01"/>
            <geometry>
                <box size="0.254 0.08 0.02"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz="0.127 0.0 -0.01"/>
            <geometry>
                 <box size="0.304 0.08 0.02"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="0.023" radius="0.05" length="0.03">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>
</robot>
```
## 7.4. Servo simulated y servo urdf

Por último, se tienen dos _xacros_ para las dos aplicaciones previamente mencionadas del robot

### 7.4.1. Servo simulated

Este _xacro_ se emplea únicamente para simular el servo en _Gazebo_, por lo que, en este archivo se incluyen los archivos necesarios para la simulación; es decir, `servo.description.xacro` y `servo.gazebo.xacro`

```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="servo">

    <xacro:include filename="$(find servo_hardware)/urdf/servo.description.xacro" />
    <xacro:include filename="$(find servo_hardware)/gazebo/servo.gazebo.xacro" />
</robot>
```

### 7.4.2. Servo urdf 

Y por otro lado, el `servo.urdf.xacro` incluye los archivos necesarios para realizar al aplicación con el servo real (`servo.description.xacro` y `servo.ros2_control.xacro`)

```xml 
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="servo">

    <xacro:include filename="$(find servo_hardware)/urdf/servo.description.xacro" />
    <xacro:include filename="$(find servo_hardware)/ros2_control/servo.ros2_control.xacro" />

    <xacro:servo_ros2_control name="servo" prefix="" />

</robot>
```