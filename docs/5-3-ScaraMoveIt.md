# 9. Scara Moveit  

Para utilizar _MoveIt_ se requieren múltiples archivos para su funcionamiento, estos se enlistan a continuación:


## 9.1 Archivos xacro

1. `gazebo.xacro`
2. `mock.xacro`
3. `servo.urdf`
4. `servo.srdf`

Los archivos xacro a excepción del archivo `scara.srdf`, son los mismos previamente realizados y explicados, por lo que estos 3 archivos se basan simplemente en incluir los demás de la siguiente manera: 

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="scara">
    
    <!-- Import scara urdf file -->
    <xacro:include filename="$(find scara)/urdf/urdf.xacro" />

    <!-- Import servo ros2_control description -->
    <xacro:include filename="$(find scara_hardware)/config/ros2_control.xacro" />
    <xacro:hardware_component 
        name="hardware_component" 
        plugin="scara_hardware/ScaraHardwareComponent" 
    />

</robot>
```

Este es el código del archivo `scara.urdf` en donde meramente se incluye el urdf del servo y se incluye la descripción del hardware del robot. 

### Scara.srdf

El único archivo que no se había mencionado hasta el momento es el `scara.srdf`, este archivo es una descripción de la _cadena_ del robot; es decir, cómo está conectado el mismo por componentes. 

```xml
<?xml version="1.0" ?>
<!--This does not replace URDF, and is not an extension of URDF.
    This is a format for representing semantic information about the robot structure.
    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
-->
<robot name="scara">
    <!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->
    <!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included-->
    <!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included-->
    <!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group-->
    <!--SUBGROUPS: Groups can also be formed by referencing to already defined group names-->
    <group name="chain">
        <link name="world"/>
        <link name="base_link"/>
        <joint name="fijacion"/>
        <link name="antebrazo"/>
        <joint name="hombro"/>
        <link name="brazo"/>
        <joint name="codo"/>
        <link name="mano"/>
        <joint name="falange"/>
        <link name="dedo"/>
        <joint name="muneca"/>
        <chain base_link="world" tip_link="dedo" />
    </group>
    <!--GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'-->
    <group_state name="cero" group="chain">
        <joint name="falange" value="0" />
        <joint name="codo" value="0" />
        <joint name="hombro" value="0" />
        <joint name="muneca" value="0" />
    </group_state>
    <group_state name="izquierda" group="chain">
        <joint name="falange" value="0" />
        <joint name="codo" value="-0.8852" />
        <joint name="hombro" value="-0.538" />
        <joint name="muneca" value="0.2" />
    </group_state>
    <group_state name="derecha" group="chain">
        <joint name="falange" value="0" />
        <joint name="codo" value="2.2737" />
        <joint name="hombro" value="2.482" />
        <joint name="muneca" value="0.2" />
    </group_state>
    <group_state name="adelante" group="chain">
        <joint name="falange" value="0" />
        <joint name="codo" value="-2.3778" />
        <joint name="hombro" value="0.8505" />
        <joint name="muneca" value="0.2" />
    </group_state>
    <group_state name="pick" group="chain">
        <joint name="falange" value="0" />
        <joint name="codo" value="-2.3778" />
        <joint name="hombro" value="0.8505" />
        <joint name="muneca" value="0" />
    </group_state>
    <!--VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)-->
    <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="world" />
    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->
    <disable_collisions link1="antebrazo" link2="base_link" reason="Adjacent" />
    <disable_collisions link1="antebrazo" link2="brazo" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="mano" reason="Never" />
    <disable_collisions link1="brazo" link2="dedo" reason="Never" />
    <disable_collisions link1="brazo" link2="mano" reason="Adjacent" />
    <disable_collisions link1="dedo" link2="mano" reason="Adjacent" />
</robot>
```

Como se puede observar, primero se enlista el _grupo_ o _cadena_ del robot bajo el nombre `group name` en donde se enlista toda la cadena de _joints_ y _links_ del robot. Luego de esto se definen los `group states` que son básicamente algunos estados del robot, estas vendrían siendo unas posiciones determinadas para el movimiento del mismo; y por último se pueden definir dos cosas `virtual joint` en donde se establece un joint entre el robot y el mundo para que no _vuele_ el robot en la simulación y `disable collisions` en donde se definen que joints no pueden _chocar_ por ser _adyacentes_ o simplemente ser _imposible_ su colisión.

## 9.2 Archivos yaml


1. `chomp_planning.yaml`
2. `ompl_planning.yaml`
3. `pliz_cartesian_limits.yaml`
4. `sensors.yaml` 
5. `moveit_controllers.yaml`
6. `joint_limits.yaml`
7. `kinematics.yaml`

Anteriormente, los archivos `yaml` podían ser extraídos con el  _moveit assistant_ en `ROS 1`, en el caso del servo y el scara se utilizaron los archivos extraídos previamente con ROS y se adaptaron a las necesidades propias. En _MoveIt_ un robot se puede mover con diferentes _planners_, tales como `chomp`, `ompl` y `pliz`, estos se atribuyen a los 3 primeros archivos `yaml`, cada uno se diferencia en el objetivo que se requiera, por ejemplo para un movimiento con coordenadas vendría siendo mejor un `pliz`; sin embargo, en el proyecto inicialmente se utilizó el `ompl`.

El archivo de `sensors.yaml` actualmente se encuentra vacío teniendo en cuenta que no se tienen sensores integrados al robot, el archivo de `moveit_controllers.yaml` se integra el controlador default de moveit y el que utilizamos para controlar el servo; es decir, el `Joint trajectory controller ` de la siguiente manera: 

### Moveit controllers
```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - scara_controller

  scara_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hombro
      - codo
      - muneca
      - falange
```

Por último se tienen los archivos `joint_limits.yaml` y `kinematics.yaml`, para `joint_limits.yaml` se decriben los límites de los joints en términos meramente de velocidad y aceleración (si aplica):

### Joint limits
```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed
# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  falange:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  codo:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  hombro:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  muneca:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
```
Y para `kinematics.yaml` se utiliza un plugin de moveit para su funcionamiento: 

### Kinematics
```yaml
group_name:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.015
```