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

#ifndef DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_

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

#include "diffdrive_arduino/arduino_comms.hpp"
#include "diffdrive_arduino/wheel.hpp"

namespace diffdrive_arduino
{
class DiffDriveArduinoHardware : public hardware_interface::SystemInterface  
// SystemInterface lets us read and write to our hardware. And inheriting from it gives us a 
// shell of a plugin that we need to fill in individual functions for. So, we will modify (override) 
// functions that are already living inside SystemInterface.
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware);

  // on_init() is the first of the state transition functions we saw earlier.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // export_state_interfaces() and export_command_interfaces() will give us the list of interfaces
  // that are available so that the rest of ros2_control knows what it has to read and write to.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // on_configure() is another state transition function we saw earlier
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // on_cleanup() is another state transition function we saw earlier
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // on_activate() is another state transition function we saw earlier
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // on_deactivate() is another state transition function we saw earlier
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // read() and write() are the most important functions. These are the actual commands 
  // that we are sending to and from our hardware to control and monitor it.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;  // This object will handle all the communications to and from the arduino. And we want to call its functions inside the state transition functions
  Config cfg_;  // This object was created from the structure we added to input our paramters that came from our xacro file in a neater way
  Wheel wheel_l_;
  Wheel wheel_r_;

};

}  // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
