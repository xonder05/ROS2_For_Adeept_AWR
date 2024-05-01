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

#ifndef ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN__DIFFBOT_SYSTEM_HPP_
#define ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <unistd.h>

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

#include "adeept_awr_diffdrive_control_plugin/visibility_control.h"
#include "adeept_awr_diffdrive_control_plugin/pwm_generator.hpp"

namespace adeept_awr_diffdrive_control_plugin
{
class AdeeptDiffDriveHardware : public hardware_interface::SystemInterface
{

struct Sides {
  int enable_pin = -1;
  int forward_pin = -1;
  int backward_pin = -1;
  double max_motor_rotation_speed = -1.0;

  struct gpiod_line *enable_line;
  struct gpiod_line *forward_line;
  struct gpiod_line *backward_line;

  PWMGenerator pwm_gen;

  double front_wheel_velocity = 0.0;
  double rear_wheel_velocity = 0.0;
  double front_wheel_position = 0.0;
  double rear_wheel_position = 0.0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AdeeptDiffDriveHardware);

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  struct gpiod_chip *chip;
  Sides left_side;
  Sides right_side;
};

}  // namespace adeept_awr_diffdrive_control_plugin

#endif  // ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN__DIFFBOT_SYSTEM_HPP_
