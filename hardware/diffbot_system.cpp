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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)  // from the passed hardware_interface::HardwareInfo & info variable, we'll be able to retrieve all the parameters that are set in the diffbot.ros2_control.xacro file.
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Filling the structure with the data coming from the parameters of our ros2_control xacro file (diffbot.ros2_control.xacro)
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
  cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
  cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);


  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// export_state_interfaces() and export_command_interfaces() tell the rest of ros2_control what 
// our hardware interface is accessible (what can be commanded and what can be monitored). Also,
// this will link each of these interfaces with the variables inside the class.

// Anytime the position and velicity of each wheel is updated, then the rest of the ros2_control system will be informed of these updates (we will make use of that in the read and write functions)
std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

// Anytime ros2_control sets velocities to the wheels, then wheel_l_.cmd and wheel_r_.cmd will be updated (we will make use of that in the read and write functions)
std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms); 
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  comms_.disconnect(); 
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  comms_.set_pid_values(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d, cfg_.pid_o);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Potential improvement to the code: ensure that we are connected before reading values with read_encoder_values()
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);  // read_encoder_values() takes in the 2 variables by reference. So, it will save the encoder values into those variables.

  // After reading the encoder values, we want to turn those encoder values into wheel position (in rads) and velocity (rad/s)
  
  // Getting the elapsed time between 2 consecutive readings (we actually have this, we just have to convert the value to seconds)
  double delta_seconds = period.seconds();

  // For left wheel
  double pos_prev = wheel_l_.pos;  // storing previous wheel positions (useful for estimating the velocities with numerical derivation)
  wheel_l_.pos = wheel_l_.calc_enc_angle();  // Calculating left wheel position. // Potential improvement to the code: handle the case of overflow where the encoder counts wrap around.
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;  // Calculating left wheel velocity  // Potential improvement to the code: use a better velocity estimate calculation.

  // For right wheel
  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Reminder that the hardware will take the motor values in encoder counts per arduino loop. So, let's convert that number. (wheel_l_.cmd is in rad/s)
  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino


// This tells ROS2 how to export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
