# 8. Scara Controllers  

Los controladores son la parte más esencial de todo el proceso de simulación y por supuesto, de control. Para toda la simulación se emplean distintos archivos de controladores, los cuales se expresan en `.yaml`. Para la simulación solamente se necesiten dos archivos de controladores: 

1. `ros2_controllers.yaml` ubicado en la carpeta `config` del paquete `scara` 
2. `moveit_controllers.yaml` ubicado en la carpeta `config` del paquete `scara_moveit_config`

Como se pudo visualizar en los archivos `xacro`, los controladores se ingresan mediante un archivo de este tipo y dependerá de la simulación que se desee realizar. Además de esto, para la selección del controlador se deben tener en cuenta varios detalles:

1. `command interfaces` del robot
2. `state interfaces` del robot
3. Controladores a disposición para adaptar ([Repositorio](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html))


## 8.1 Estructura de archivo  

Los archivos de controlares están compuestos por dos partes: _Declaración de controlares_ y _Descripción de controladores_


### Declaración de controladores

Para el caso del servo se utilizaron los controladores: `Joint trajectory controller ` y `Joint state broadcaster`, el primero de estos sirve para darle una trayectoria de _puntos_ al controlador y esté lo utilizará para mover robot y alcanzar estos puntos. Por otro lado, el `Joint state broadcaster` se utiliza para poder tener una conexión continua con el estado de los joints del robot, es recomedable utilizarlo siempre en estos casos. 


```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    scara_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

```

La declaración como se puede visualizar esta compuesta meramente por el _nombramiento_ de los controlares, utilizando la librería y nombre indicado.


### Descripción de controladores

Después de esto, la descripción se basa en el ingreso de todos los parámetros que necesita el controlador para funcionar; es decir, se ingresan los `joints`, `command interfaces` y `state interfaces`, claramente si el controlador lo requiere.

```yaml
scara_controller:
  ros__parameters:
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - hombro
      - codo
      - muneca
      - falange

joint_state_broadcaster:
  ros__parameters:
    joints:
      - hombro
      - codo
      - muneca
      - falange
```
En este caso el `Joint trajectory controller ` requiere el ingreso de todos los parámetros mencionados, pero por otro lado el `Joint state broadcaster` solamente necesita los  `joints`. 