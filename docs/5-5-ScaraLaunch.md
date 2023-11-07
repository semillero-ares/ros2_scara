# 11. Scara launch


Los archivos Launch son de vital importancia porque son con los que se ejecuta todo un paquete. Estos archivos normalmente se hacen en python y su funcionamiento es bastante básico. Su arquitectura se constituye de una función general llamada `generate_launch_description`  que internamente posee los definiciones con parámetros de los diferentes nodos que deseamos ejecutar. como Gazebo, Rviz, joint_state_publisher_gui, etc. Y al final, posee un return LaunchDescription con los nodos que se desean ejecutar en ese Launch.

En este repositorio se encuentran distintos `Launch` descritos a continuación:

1. **demo.launch.py:** Archivo `Launch` general para realizar el control con el Scara _REAL_
2. **gazebo.launch.py:** Archivo `Launch` para realizar el control del scara en la simulación con `Gazebo`
3. **mock.launch.py:** Archivo `Launch`para realizar el control en `Rviz` con un scara _FALSO_ 

Estos 3 archivos `Launch` son necesarios para toda la ejecución y control del scara ya que son como una _secuencia_ para el éxito del proyecto. Primero se debe asegurar que el `mock.launch.py` funcione correctamente, después de esto se debe lograr que el `gazebo.launch.py` funcione para así pasar con el _Hardware Component_ al `demo.launch.py`.

## 11.1. Mock Launch

El archivo `mock.launch.py` es uno de los más importantes porque es el primer paso para el control de cualquier dispositivo. Al funcionar este archivo se está asegurando que el planner de `moveit` está funcionando correctamente en conjunto con el `URDF` y los `xacros`.

Al inicio de cualquier `Launch` lo primero que se tiene es la importación de funciones necesarias 

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
```

Luego empieza la definición de los nodos del `Launch`, comenzando con `moveit_config` y `run_move_group_node` para la configuración del _planner_: 
```python
def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    moveit_config = (
        MoveItConfigsBuilder("scara")
        .robot_description(file_path="config/mock.xacro")
        .robot_description_semantic(file_path="config/scara.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
```
Luego sigue la configuración del `Rviz`
```python
    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("scara_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        on_exit=Shutdown()
    )
```

Luego se ingresa la posición inicial del robot y la definición del `robot_state_publisher` para realizar las transformaciones necesarias con el movimiento
```python
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id","world", 
            "--child-frame-id","base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
```

Luego se ingresa el nodo de `ros2_control` con la ruta de los controladores necesarios 
```python
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("scara"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )
```

Y la ejecución de los controladores del robot
```python
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    scara_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "scara_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )
```
Y como se mencionó al inicio, siempre al final del `Launch` se llaman a los nodos que se desean activar
```python
    return LaunchDescription(
        [
            db_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
            joint_state_broadcaster_spawner,
            scara_controller_spawner,
        ]
    )
```

## 11.2. Gazebo Launch

En constitución, todos los launch son muy similares, se diferencian en los parámetros de los distintos nodos que contienen o que por su funcionamiento óptimo se añade un nodo adicional. En el caso del  `gazebo.launch.py`, tiene las siguientes diferencias:

Primero en la importación de funciones:
```python
import os
from launch import LaunchDescription
from launch.actions import Shutdown, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
```

Luego, tiene un nodo para la ejecución de `Gazebo` 
```python
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
```
Para la simulación se habilita el tiempo de la misma con el parámetro `use_sim_time`
```python
    use_sim_time = {"use_sim_time": True}
    moveit_config = (
        MoveItConfigsBuilder("scara")
        .robot_description(file_path="config/gazebo.xacro")
        .robot_description_semantic(file_path="config/scara.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
```

Se añade un nodo para spawnear la entidad en el entorno de simulación de `Gazebo`
```python
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scara'],
                        output='screen')
```

Y por último entre la ejecución de los controladores se añade un _delay_ para evitar un fallo entre los mismos
```python
    #NODO DE JOINT_STATE_BROADCASTER

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
     
    #NODO DE SCARA_CONTROLLER

    delayed_scara_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[scara_controller_spawner],
        )
    )
```

## 11.3. Demo Launch

El demo Launch es el archivo que se utiliza para controlar con el servo _REAL_ conectado. Curiosamente, es un `Launch` muy similar al `mock.launch.py` y solo existe una diferencia entre estos dos archivos:

```python
    moveit_config = (
        MoveItConfigsBuilder("scara")
        .robot_description(file_path="config/scara.urdf")
        .robot_description_semantic(file_path="config/scara.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
```
En vez de tomar `mock.xacro`, se toma el `scara.urdf` de la carpeta config. Con esto ya no se estaría tomando el plugin de un scara _FALSO_ sino el plugin constituido en _C++_ para el scara _REAL_.