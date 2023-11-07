# 10. Scara hardware  

El _Hardware Component_ del Scara se compone de lo siguiente: 

1. Archivo `scara_system.cpp` en donde se define el sistema de hardware del robot, incluyendo la configuración de controladores, la comunicación de sensores y motores del robot (si aplica) y funciones específicas 
2. Carpeta `include` donde se almacenan archivos de encabezado `.h` y `.hpp` en donde se declaran funciones, clases y estructuras utiliadas en el código. 


Esta carpeta contiene 4 archivos importantes: 

1. `arduino_comms.hpp` en donde se contienen las definiciones y declaraciones relacionadas con la comunicación entre el Scara y arduino
2. `joint.hpp` en donde se define la clase o las estructuras relacionadas con los joints del robot
3. `scara_systema.hpp` en donde se define la clase principal o estructura que representa el sistema del robot
4. `visibility_control.h` que se utiliza para la exportación de símbolos en bibliotecas y asegurar así su correcta compilación y vinculación. 

A continuación se realiza una descripción de los archivos que componen el _Hardware Component_

## 10.1 scara_system.cpp 

Primero se incluyen todas las librerías necesarias para la definición del _sistema_ del robot 

```c++
#include "scara_hardware/scara_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
```


Después de esto se define un espacio de nombres llamado `scara_hardware` con un condicional inicial que se asegura de la correcta inicialización del robot

```c++
namespace scara_hardware
{
  hardware_interface::CallbackReturn ScaraHardwareComponent::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
```
Luego de esto se definen todos los parámetros necesarios para cada joint de la siguiente manera: 

```c++
// Joint 1
    std::string j1_name = info_.hardware_parameters["joint1_name"];
    double j1_gain = std::stof(info_.hardware_parameters["joint1_controller_gain"]);
    double j1_counts = std::stof(info_.hardware_parameters["joint1_counts_per_unit"]);
    double j1_ref = std::stof(info_.hardware_parameters["joint1_ref"]);
    int j1_mintp = std::stoi(info_.hardware_parameters["joint1_minimum_time_period"]);
    int j1_maxtp = std::stoi(info_.hardware_parameters["joint1_maximum_time_period"]);
    int j1_offset = std::stoi(info_.hardware_parameters["joint1_offset"]);

    joint1_.setup(j1_name, j1_gain, j1_counts, j1_ref, j1_mintp, j1_maxtp, j1_offset);
```

Además, se ingresan los parámetros para la comunicación serial 

```c++
cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.echo = info_.hardware_parameters["echo"] == "true" || info_.hardware_parameters["echo"] == "1";
```

Después de esto se realiza un bucle para confirmar la información de los joints contenida en las `command_interfaces` y `state_interfaces`. Por ejemplo, a continuación se confirma que las  `command_interfaces` sea sólo _UNA_: de  `POSITION` (esto debe tener concordancia con la definición de los controladores del robot). 

```c++
for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ScaraHardwareComponent"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ScaraHardwareComponent"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
```
De igual manera se realiza con las `state_interfaces` confirmando que sean dos: `POSITION` y `VELOCITY`

```c++
if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ScaraHardwareComponent"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ScaraHardwareComponent"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ScaraHardwareComponent"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }
```

Luego se crea un vector para exportar las `command_interfaces` y `state_interfaces`  de los joints del robot

```c++
std::vector<hardware_interface::StateInterface> ScaraHardwareComponent::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint1_.name, hardware_interface::HW_IF_POSITION, &joint1_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint1_.name, hardware_interface::HW_IF_VELOCITY, &joint1_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint2_.name, hardware_interface::HW_IF_POSITION, &joint2_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint2_.name, hardware_interface::HW_IF_VELOCITY, &joint2_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint3_.name, hardware_interface::HW_IF_POSITION, &joint3_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint3_.name, hardware_interface::HW_IF_VELOCITY, &joint3_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint4_.name, hardware_interface::HW_IF_POSITION, &joint4_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint4_.name, hardware_interface::HW_IF_VELOCITY, &joint4_.velocity));

    return state_interfaces;
  }


   std::vector<hardware_interface::CommandInterface> ScaraHardwareComponent::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint1_.name, hardware_interface::HW_IF_POSITION, &joint1_.ref));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint2_.name, hardware_interface::HW_IF_POSITION, &joint2_.ref));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint3_.name, hardware_interface::HW_IF_POSITION, &joint3_.ref));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint4_.name, hardware_interface::HW_IF_POSITION, &joint4_.ref));

    return command_interfaces;
  }
```

Luego se realiza la creación de funciones como `on_configure`, `on_cleanup`, `on_activate` y `on_deactivate`

```c++
hardware_interface::CallbackReturn ScaraHardwareComponent::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Configuring ...please wait...");
    // RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), std::string(cfg_.device));
    // RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), std::string(cfg_.baud_rate));
    // RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), std::string(cfg_.timeout_ms));
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ScaraHardwareComponent::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ScaraHardwareComponent::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ScaraHardwareComponent::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("ScaraHardwareComponent"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }
```

Luego de esto, se crea la función `read` que tiene como objetivo _leer_ los datos desde el hardware del robot SCARA y actualizar las variables internas que representan el estado de las articulaciones. Primero se verifica si la comunicación con el hardware está establecida.

```c++
hardware_interface::return_type ScaraHardwareComponent::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
```

Luego se crea un string con toda la información de las articulaciones en un archivo .CSV llamado `ss`

```c++
double delta_seconds = period.seconds();

    std::stringstream ss;
    ss << joint1_.direction_cmd << ",";
    ss << joint1_.period_cmd << ",";
    ss << joint2_.direction_cmd << ",";
    ss << joint2_.period_cmd << ",";
    ss << joint3_.direction_cmd << ",";
    ss << joint3_.period_cmd << ",";
    ss << joint4_.direction_cmd << ",";
    ss << joint4_.period_cmd << "\n";

    std::string response = comms_.send_msg(ss.str(),cfg_.echo);
```

Luego de almacenada esta información se procesa esta respuesta para ser almacenadas en otras variables

```c++
 if (response.length() > 0)
    {
      std::string delimiter = ",";
      size_t del_pos = response.find(delimiter);

      std::string aux_value = response.substr(0, del_pos);
      joint1_.position_cnts = std::atoi(aux_value.c_str());
      response = response.substr(del_pos + delimiter.length());

      del_pos = response.find(delimiter);
      aux_value = response.substr(0, del_pos);
      joint2_.position_cnts = std::atoi(aux_value.c_str());
      response = response.substr(del_pos + delimiter.length());

      del_pos = response.find(delimiter);
      aux_value = response.substr(0, del_pos);
      joint3_.position_cnts = std::atoi(aux_value.c_str());
      response = response.substr(del_pos + delimiter.length());

      joint4_.position_cnts = std::atoi(response.c_str());
    }
```

Después se utiliza el método `loop` para actualizar el estado de las articulaciones con base en la nueva información obtenida y el tiempo transcurrido.

```c++
    joint1_.loop(delta_seconds);
    joint2_.loop(delta_seconds);
    joint3_.loop(delta_seconds);
    joint4_.loop(delta_seconds);

    return hardware_interface::return_type::OK;
  }

```


Luego, se escribe la función `write`, que lo único que hace es devolver un `ok` en el caso de estar conectado con el hardware del robot
```c++
hardware_interface::return_type scara_hardware ::ScaraHardwareComponent::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

} 
```

Y por último, se incluye un plugin para exportar las clases necesarias, en este caso `scara_hardware` como `ScaraHardwareComponent`  y `hardware_interface` como `SystemInterface`.

```c++
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    scara_hardware::ScaraHardwareComponent, hardware_interface::SystemInterface)
```
## 10.2 scara_system.hpp 

Primero se incluyen las librerías y archivos necesarios; además de definir el símbolo `SCARA_HARDWARE__SERVO_SYSTEM_HPP_`


```c++
#ifndef SCARA_HARDWARE__SERVO_SYSTEM_HPP_
#define SCARA_HARDWARE__SERVO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "scara_hardware/visibility_control.h"
#include "scara_hardware/arduino_comms.hpp"
#include "scara_hardware/joint.hpp"
```

Se define un espacio llamado `scara_hardware` junto con la clase `ScaraHardwareComponent`; además, se crea una estructura llamada `Config` en donde se almacena la configuración y los parámetros relacionados con el hardware del robot, como `loop_rate`, `baud_rate`, `timeout_ms`, etc. 
```c++
namespace scara_hardware
{
class ScaraHardwareComponent : public hardware_interface::SystemInterface
{

struct Config
{
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  bool echo = false;
};
```

Luego, se validan funciones como `on_init`, `on_configure`, `export_state_interfaces`, `read`, etc.

```c++
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(ScaraHardwareComponent);

  SCARA_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SCARA_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SCARA_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SCARA_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
```


Por último, se definen miembros utilizados en la clase descrita como `ArduinoComms`, `Joint`, `Config`, etc. Además, se finaliza el espacio `scara_hardware` y la definición del símbolo `SCARA_HARDWARE__SERVO_SYSTEM_HPP_`.

```c++
private:

  ArduinoComms comms_;
  Config cfg_;
  Joint joint1_;
  Joint joint2_;
  Joint joint3_;
  Joint joint4_;

};

}  // namespace scara_hardware

#endif  // SCARA_HARDWARE__SERVO_SYSTEM_HPP_
```
## 10.3 arduino_comms.hpp


El archivo `arduino_comms.hpp` se basa simplemente en recibir todos los parámetros pertinentes para el arduino, primero se incluyen las librerías como en los demás archivos

```c++
#ifndef SCARA_HARDWARE_ARDUINO_COMMS_HPP
#define SCARA_HARDWARE_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
```
Se define la función `convert_baud_rate` 


```c++
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}
```

Luego se empieza con la creación de la clase `ArduinoComms` y funciones internas como `connect`, `disconnect`, `connected`, etc.

```c++
class ArduinoComms
{

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }
```


Después se crea la función `send_msg` para enviar un mensaje a través de una conexión serial y recibir una respuesta
```c++
std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "\e[1;31m The ReadByte() call has timed out. \e[0m" << std::endl;
    }

    if (print_output)
    {
      std::string msg_sent = msg_to_send.substr(0, msg_to_send.length() - 1);
      std::cout << "\e[1;32m Sent: " << msg_sent << "\e[0m\n\e[1;32m Recv: " << response << "\e[0m" << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }
```
Con esto se almacena información en la variable `response` en un tiempo establecido (`timeout_ms_`), esta variable se utiliza en el archivo `scara_system.cpp`. Y con esto se finaliza el archivo con los miembros privados
```c++
private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // SCARA_HARDWARE_ARDUINO_COMMS_HPP
```



## 10.4 joint.hpp

Ahora con toda la definición de la clase `Joint`, primero como con los demás documentos se incluyen las librerías y archivos

```c++
#ifndef SCARA_HARDWARE_JOINT_HPP
#define SCARA_HARDWARE_JOINT_HPP
#include <string>
#include <cmath>
```

Se definen las variables con su clase pertinente dentro de la inicialización de la clase 


```c++
class Joint
{
    public:

    std::string name = "";
    double ref = 0;
    double error = 0;
    double gain = 0; // <------------- input parameter
    double cmd = 0; 
    double counts_per_unit = 0; // <-- input parameter
    double velocity_cmd = 0;
    int period_cmd = 0;
    int direction_cmd = 0;
    int us_min = 1; // <-------------- input parameter
    int us_max = 10; // <------------- input parameter
    int position_cnts = 0;
    int position_offset = 0; // <----- input parameter
    double position = 0;
    double velocity = 0;
```

Luego se ingresan todos los parámetros de la clase `Joint`


```c++
Joint() = default;

    Joint(
      const std::string &joint_name, 
      double controller_gain, 
      double counts, 
      double reference,
      int min, 
      int max, 
      int offset)
    {
      setup(joint_name, controller_gain, counts, reference, min, max, offset);
    }
```

Junto con las funciones iniciales para buscar el control óptimo del robot 

```c++
void setup(
      const std::string &joint_name, 
      double controller_gain, 
      double counts, 
      double reference,
      int min, 
      int max, 
      int offset)
    {
      // M_PI es la variable de pi=3.14159
      name = joint_name;
      gain = controller_gain;
      counts_per_unit = counts;
      ref = reference;
      us_min = min;
      us_max = max;
      position_offset = offset;
    }

    void loop(double dt){
      measure(dt);
      control();
    }
```

Con esto se definen todos los miembros privados de la clase en donde se definen distintas funciones para realizar el control óptimo de las posiciones de las articulaciones del robot  para finalizar el archivo. 

```c++
private: 

    int position_delta_ = 0;

    double calc_position(){
      position_delta_ = position_cnts - position_offset;
      return (double) position_delta_ / counts_per_unit;
    }

    void measure(double dt){
      double pos_prev = position;
      position = calc_position();
      velocity = (position - pos_prev) / dt;
    }

    void control(){
      error = ref - position;
      cmd = gain*error;
      velocity_cmd = cmd*counts_per_unit;
      if(velocity_cmd == 0){
        period_cmd = 0;
      }else{
        if(velocity_cmd>0){
          direction_cmd = 1;
        }else{
          direction_cmd = 0;
        }
        period_cmd = (int) (1000000/velocity_cmd);
        period_cmd = fmin(fmax(abs(period_cmd),us_min),us_max);
      }
    }

};

#endif // SCARA_HARDWARE_JOINT_HPP
```


## 10.5 visibility_control.h
Por último el archivo `visibility_control.h`, este archivo de encabezado  se utiliza para definir directivas de preprocesador que gestionan la visibilidad de los símbolos (funciones, variables, clases, etc.) en una biblioteca o módulo de código. La visibilidad de los símbolos se refiere a si esos símbolos son accesibles desde fuera de la biblioteca o módulo.


```c++
#ifndef SCARA_HARDWARE__VISIBILITY_CONTROL_H_
#define SCARA_HARDWARE__VISIBILITY_CONTROL_H_


#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SCARA_HARDWARE_EXPORT __attribute__((dllexport))
#define SCARA_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define SCARA_HARDWARE_EXPORT __declspec(dllexport)
#define SCARA_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef SCARA_HARDWARE_BUILDING_DLL
#define SCARA_HARDWARE_PUBLIC SCARA_HARDWARE_EXPORT
#else
#define SCARA_HARDWARE_PUBLIC SCARA_HARDWARE_IMPORT
#endif
#define SCARA_HARDWARE_PUBLIC_TYPE SCARA_HARDWARE_PUBLIC
#define SCARA_HARDWARE_LOCAL
#else
#define SCARA_HARDWARE_EXPORT __attribute__((visibility("default")))
#define SCARA_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define SCARA_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define SCARA_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define SCARA_HARDWARE_PUBLIC
#define SCARA_HARDWARE_LOCAL
#endif
#define SCARA_HARDWARE_PUBLIC_TYPE
#endif

#endif  // SCARA_HARDWARE__VISIBILITY_CONTROL_H_
```



