// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include <netdb.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "controlko_hardware_interface/rrbot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controlko_hardware_interface
{
hardware_interface::CallbackReturn RRBotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_acc_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  gpio_commands_.resize(
    info_.gpios[0].command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  gpio_states_.resize(
    info_.gpios[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  gpio_ins_storage_.resize(2, false);
  gpio_outs_storage_.resize(2, false);
  gpio_cmds_storage_.resize(2, false);
  sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rrbot_comms_ = std::make_unique<dr_denis_rrbot_comms::DrDenisRRBotComms>(info_.joints.size());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RRBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_acc_states_[i]));
  }

  size_t i = 0;
  for (const auto & state_itf : info_.gpios[0].state_interfaces)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.gpios[0].name, state_itf.name, &sensor_states_[i]));
    ++i;
  }

  i = 0;
  for (const auto & state_itf : info_.sensors[0].state_interfaces)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.sensors[0].name, state_itf.name, &gpio_states_[i]));
    ++i;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 2);
  position_command_interface_names_.reserve(info_.joints.size());
  velocity_command_interface_names_.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_commands_[i]));
    position_command_interface_names_.push_back(command_interfaces.back().get_name());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_commands_[i]));
    velocity_command_interface_names_.push_back(command_interfaces.back().get_name());
  }

  size_t i = 0;
  for (const auto & command_itf : info_.sensors[0].command_interfaces)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.sensors[0].name, command_itf.name, &gpio_states_[i]));
    ++i;
  }

  return command_interfaces;
}

hardware_interface::return_type RRBotHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  bool starting_at_least_one_pos_itf = false;
  bool starting_at_least_one_vel_itf = false;

  for (const auto & itf : position_command_interface_names_)
  {
    auto find_it = std::find(start_interfaces.begin(), start_interfaces.end(), itf);
    if (find_it != start_interfaces.end())
    {
      starting_at_least_one_pos_itf = true;
    }
  }
  for (const auto & itf : velocity_command_interface_names_)
  {
    auto find_it = std::find(start_interfaces.begin(), start_interfaces.end(), itf);
    if (find_it != start_interfaces.end())
    {
      starting_at_least_one_vel_itf = true;
    }
  }

  // if (starting_at_least_one_pos_itf && starting_at_least_one_vel_itf)
  // {
  //   return hardware_interface::return_type::ERROR;
  // }

  bool stopping_all_pos_itfs = true;
  bool stopping_all_vel_itfs = true;

  for (const auto & itf : position_command_interface_names_)
  {
    auto find_it = std::find(stop_interfaces.begin(), stop_interfaces.end(), itf);
    if (find_it == stop_interfaces.end())
    {
      stopping_all_pos_itfs = false;
    }
  }
  for (const auto & itf : velocity_command_interface_names_)
  {
    auto find_it = std::find(stop_interfaces.begin(), stop_interfaces.end(), itf);
    if (find_it == stop_interfaces.end())
    {
      stopping_all_vel_itfs = false;
    }
  }

  auto current_control_mode = rrbot_comms_->get_control_mode();

  // want to use position mode
  if (
    starting_at_least_one_pos_itf && !starting_at_least_one_vel_itf ||
    (starting_at_least_one_pos_itf &&
     (current_control_mode == dr_denis_rrbot_comms::control_mode_type::VELOCITY &&
      stopping_all_vel_itfs)))
  {
    switch_to_mode_ = dr_denis_rrbot_comms::control_mode_type::POSITION;
  }

  if (
    starting_at_least_one_vel_itf && !starting_at_least_one_pos_itf ||
    (starting_at_least_one_vel_itf &&
     (current_control_mode == dr_denis_rrbot_comms::control_mode_type::POSITION &&
      stopping_all_pos_itfs)))
  {
    switch_to_mode_ = dr_denis_rrbot_comms::control_mode_type::POSITION;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  rrbot_comms_->set_control_mode(switch_to_mode_);

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rrbot_comms_->init();

  // set some default values for joints
  rrbot_comms_->read_joint_states(joint_pos_states_, joint_vel_states_, joint_acc_states_);

  joint_pos_commands_ = joint_pos_states_;
  joint_vel_commands_ = joint_vel_states_;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rrbot_comms_->stop();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rrbot_comms_.release();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rrbot_comms_->read_joint_states(joint_pos_states_, joint_vel_states_, joint_acc_states_);
  rrbot_comms_->read_sensor_values(sensor_states_);

  rrbot_comms_->read_gpio(gpio_ins_storage_, gpio_outs_storage_);
  for (size_t i = 0; i < 2; ++i)
  {
    gpio_states_[i] = static_cast<double>(gpio_ins_storage_[i]);
    gpio_states_[2 + i] = static_cast<double>(gpio_outs_storage_[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rrbot_comms_->write_joint_commands(joint_pos_commands_, joint_vel_commands_);

  for (size_t i = 0; i < info_.gpios[0].command_interfaces.size(); ++i)
  {
    gpio_cmds_storage_[i] = static_cast<bool>(gpio_commands_[i]);
  }
  rrbot_comms_->write_gpios(gpio_cmds_storage_);

  return hardware_interface::return_type::OK;
}

}  // namespace controlko_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  controlko_hardware_interface::RRBotHardwareInterface, hardware_interface::SystemInterface)
