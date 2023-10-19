// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scara_hardware/scara_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

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

    // Joint 1
    std::string j1_name = info_.hardware_parameters["joint1_name"];
    double j1_gain = std::stof(info_.hardware_parameters["joint1_controller_gain"]);
    double j1_counts = std::stof(info_.hardware_parameters["joint1_counts_per_unit"]);
    double j1_ref = std::stof(info_.hardware_parameters["joint1_ref"]);
    int j1_mintp = std::stoi(info_.hardware_parameters["joint1_minimum_time_period"]);
    int j1_maxtp = std::stoi(info_.hardware_parameters["joint1_maximum_time_period"]);
    int j1_offset = std::stoi(info_.hardware_parameters["joint1_offset"]);

    joint1_.setup(j1_name, j1_gain, j1_counts, j1_ref, j1_mintp, j1_maxtp, j1_offset);

    // Joint 2
    std::string j2_name = info_.hardware_parameters["joint2_name"];
    double j2_gain = std::stof(info_.hardware_parameters["joint2_controller_gain"]);
    double j2_counts = std::stof(info_.hardware_parameters["joint2_counts_per_unit"]);
    double j2_ref = std::stof(info_.hardware_parameters["joint2_ref"]);
    int j2_mintp = std::stoi(info_.hardware_parameters["joint2_minimum_time_period"]);
    int j2_maxtp = std::stoi(info_.hardware_parameters["joint2_maximum_time_period"]);
    int j2_offset = std::stoi(info_.hardware_parameters["joint2_offset"]);

    joint2_.setup(j2_name, j2_gain, j2_counts, j2_ref, j2_mintp, j2_maxtp, j2_offset);

    // Joint 3
    std::string j3_name = info_.hardware_parameters["joint3_name"];
    double j3_gain = std::stof(info_.hardware_parameters["joint3_controller_gain"]);
    double j3_counts = std::stof(info_.hardware_parameters["joint3_counts_per_unit"]);
    int j3_mintp = std::stoi(info_.hardware_parameters["joint3_minimum_time_period"]);
    int j3_maxtp = std::stoi(info_.hardware_parameters["joint3_maximum_time_period"]);
    int j3_offset = std::stoi(info_.hardware_parameters["joint3_offset"]);

    joint3_.setup(j3_name, j3_gain, j3_counts, 0.0, j3_mintp, j3_maxtp, j3_offset);

    // Joint 4
    std::string j4_name = info_.hardware_parameters["joint4_name"];
    double j4_gain = std::stof(info_.hardware_parameters["joint4_controller_gain"]);
    double j4_counts = std::stof(info_.hardware_parameters["joint4_counts_per_unit"]);
    int j4_mintp = std::stoi(info_.hardware_parameters["joint4_minimum_time_period"]);
    int j4_maxtp = std::stoi(info_.hardware_parameters["joint4_maximum_time_period"]);
    int j4_offset = std::stoi(info_.hardware_parameters["joint4_offset"]);

    joint4_.setup(j4_name, j4_gain, j4_counts, 0.0, j4_mintp, j4_maxtp, j4_offset);

    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.echo = info_.hardware_parameters["echo"] == "true" || info_.hardware_parameters["echo"] == "1";

    //

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

  hardware_interface::return_type ScaraHardwareComponent::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

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

    joint1_.loop(delta_seconds);
    joint2_.loop(delta_seconds);
    joint3_.loop(delta_seconds);
    joint4_.loop(delta_seconds);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type scara_hardware ::ScaraHardwareComponent::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

} // namespace scara_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    scara_hardware::ScaraHardwareComponent, hardware_interface::SystemInterface)