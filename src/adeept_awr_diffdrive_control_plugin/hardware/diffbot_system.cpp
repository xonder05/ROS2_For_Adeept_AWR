#include "adeept_awr_diffdrive_control_plugin/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace adeept_awr_diffdrive_control_plugin
{
hardware_interface::CallbackReturn AdeeptDiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // loading parameters
  left_side.enable_pin = std::stoi(info_.hardware_parameters["motor_left_enable_pin"]);
  left_side.forward_pin = std::stoi(info_.hardware_parameters["motor_left_forward_pin"]);
  left_side.backward_pin = std::stoi(info_.hardware_parameters["motor_left_backward_pin"]);
  left_side.max_motor_rotation_speed = std::stof(info_.hardware_parameters["max_motor_rotation_speed"]);
  right_side.enable_pin = std::stoi(info_.hardware_parameters["motor_right_enable_pin"]);
  right_side.forward_pin = std::stoi(info_.hardware_parameters["motor_right_forward_pin"]);
  right_side.backward_pin = std::stoi(info_.hardware_parameters["motor_right_backward_pin"]);
  right_side.max_motor_rotation_speed = std::stof(info_.hardware_parameters["max_motor_rotation_speed"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AdeeptDiffDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AdeeptDiffDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AdeeptDiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // there is no feedback from the motors so theese interfaces are practically useless
  // but used controller requires theirs exsitence to function
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_front_wheel_joint", hardware_interface::HW_IF_POSITION, &left_side.front_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_side.front_wheel_velocity));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_rear_wheel_joint", hardware_interface::HW_IF_POSITION, &left_side.rear_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_side.rear_wheel_velocity));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_front_wheel_joint", hardware_interface::HW_IF_POSITION, &right_side.front_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_side.front_wheel_velocity));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_rear_wheel_joint", hardware_interface::HW_IF_POSITION, &right_side.rear_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_side.rear_wheel_velocity));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AdeeptDiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // input commands from controller are passed throw them
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "left_front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_side.front_wheel_velocity));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "left_rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_side.rear_wheel_velocity));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "right_front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_side.front_wheel_velocity));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "right_rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_side.rear_wheel_velocity));

  return command_interfaces;
}

hardware_interface::CallbackReturn AdeeptDiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AdeeptDiffDriveHardware"), "Activating ...please wait...");

  // gpio initialization
  chip = gpiod_chip_open("/dev/gpiochip0");
  if (!chip) {
      std::cerr << "Error opening GPIO chip\n";
      return hardware_interface::CallbackReturn::ERROR;
  }

  left_side.enable_line = gpiod_chip_get_line(chip, left_side.enable_pin);
  left_side.forward_line = gpiod_chip_get_line(chip, left_side.forward_pin);
  left_side.backward_line = gpiod_chip_get_line(chip, left_side.backward_pin);
  
  right_side.enable_line = gpiod_chip_get_line(chip, right_side.enable_pin);
  right_side.forward_line = gpiod_chip_get_line(chip, right_side.forward_pin);
  right_side.backward_line = gpiod_chip_get_line(chip, right_side.backward_pin);

  if (!left_side.enable_line || !left_side.forward_line || !left_side.backward_line ||
      !right_side.enable_line || !right_side.forward_line || !right_side.backward_line) {
      std::cerr << "Error getting GPIO lines\n";
      gpiod_chip_close(chip);
      return hardware_interface::CallbackReturn::ERROR;
  }

  if (gpiod_line_request_output(left_side.enable_line, "example", 0) != 0 ||
      gpiod_line_request_output(left_side.forward_line, "example", 0) != 0 ||
      gpiod_line_request_output(left_side.backward_line, "example", 0) != 0 ||
      gpiod_line_request_output(right_side.enable_line, "example", 0) != 0 ||
      gpiod_line_request_output(right_side.forward_line, "example", 0) != 0 ||
      gpiod_line_request_output(right_side.backward_line, "example", 0) != 0) 
  {
      std::cerr << "Error setting GPIO lines as output\n";
      gpiod_chip_close(chip);
      return hardware_interface::CallbackReturn::ERROR;
  }

  left_side.pwm_gen.initialize(left_side.enable_line);
  left_side.pwm_gen.start();

  right_side.pwm_gen.initialize(right_side.enable_line);
  right_side.pwm_gen.start();

  RCLCPP_INFO(rclcpp::get_logger("AdeeptDiffDriveHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AdeeptDiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AdeeptDiffDriveHardware"), "Deactivating ...please wait...");

  // stopping motors before exiting
  left_side.pwm_gen.stop();
  right_side.pwm_gen.stop();

  if (gpiod_line_set_value(left_side.enable_line, 0) != 0 ||
      gpiod_line_set_value(left_side.forward_line, 1) != 0 ||
      gpiod_line_set_value(left_side.backward_line, 1) != 0 ||
      gpiod_line_set_value(right_side.enable_line, 1) != 0 ||
      gpiod_line_set_value(right_side.forward_line, 1) != 0 ||
      gpiod_line_set_value(right_side.backward_line, 0) != 0) 
  {
      std::cerr << "Error setting GPIO lines to 0\n";
  }

  gpiod_chip_close(chip);

  RCLCPP_INFO(rclcpp::get_logger("AdeeptDiffDriveHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AdeeptDiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type adeept_awr_diffdrive_control_plugin ::AdeeptDiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (left_side.front_wheel_velocity != left_side.rear_wheel_velocity || right_side.front_wheel_velocity != right_side.front_wheel_velocity)
  {
    RCLCPP_INFO(rclcpp::get_logger("AdeeptDiffDriveHardware"), "Not same speed\n");
  }

  // recieved command cut to limits of the motor and scaled to pwm freq
  double left_motor_speed = std::max(std::min(left_side.front_wheel_velocity, left_side.max_motor_rotation_speed), -left_side.max_motor_rotation_speed);
  double normalized_left_motor_speed = (left_motor_speed / (left_side.max_motor_rotation_speed + 1)) * 100;
  
  double right_motor_speed = std::max(std::min(right_side.front_wheel_velocity, right_side.max_motor_rotation_speed), -right_side.max_motor_rotation_speed);
  double normalized_right_motor_speed = (right_motor_speed / (right_side.max_motor_rotation_speed + 1)) * 100;
  
  // controlling gpio and pwm gen
  if (normalized_left_motor_speed > 0) {
    gpiod_line_set_value(left_side.forward_line, 1);
    gpiod_line_set_value(left_side.backward_line, 0);
  }
  else {
    gpiod_line_set_value(left_side.backward_line, 1);
    gpiod_line_set_value(left_side.forward_line, 0);
  }

  if (normalized_right_motor_speed > 0) {
    gpiod_line_set_value(right_side.forward_line, 1);
    gpiod_line_set_value(right_side.backward_line, 0);
  }
  else {
    gpiod_line_set_value(right_side.backward_line, 1);
    gpiod_line_set_value(right_side.forward_line, 0);
  }
  left_side.pwm_gen.set_duty_cycle(std::abs(normalized_left_motor_speed));
  right_side.pwm_gen.set_duty_cycle(std::abs(normalized_right_motor_speed));

  return hardware_interface::return_type::OK;
}

}  // namespace adeept_awr_diffdrive_control_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adeept_awr_diffdrive_control_plugin::AdeeptDiffDriveHardware, hardware_interface::SystemInterface)
