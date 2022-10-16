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

#ifndef CONTROLKO_HARDWARE_INTERFACE__DR_DENIS_RRBOT_COMMS_HPP_
#define CONTROLKO_HARDWARE_INTERFACE__DR_DENIS_RRBOT_COMMS_HPP_

#include <chrono>
#include <limits>
#include <vector>

namespace dr_denis_rrbot_comms
{

enum class control_mode_type : std::uint8_t
{
  POSITION = 0,
  VELOCITY = 1,
};

static constexpr double MAX_VELOCITY = 0.5;
static constexpr double SENSOR_MAX = 23.28;

class DrDenisRRBotComms
{
public:
  DrDenisRRBotComms(const size_t number_of_dofs)
      : nr_dofs_(number_of_dofs), current_control_mode_(control_mode_type::POSITION), movement_enabled_(false)
  {
    joint_positions_.resize(nr_dofs_, 0.1);
    joint_velocities_.resize(nr_dofs_, 0.0);
    joint_accelerations_.resize(nr_dofs_, 0.0);
    digital_outs_.resize(nr_dofs_, false);
  }

  void init()
  {
    last_time_stamp_ = std::chrono::steady_clock::now();
    movement_enabled_ = true;
  }

  void stop()
  {
    movement_enabled_ = false;
  }

  bool set_control_mode(const control_mode_type & control_mode)
  {
    current_control_mode_ = control_mode;

    return true;
  }

  control_mode_type get_control_mode() { return current_control_mode_; }

  bool write_joint_commands(
    const std::vector<double> & position_command, const std::vector<double> & velocity_command)
  {
    if (movement_enabled_)
    {
      const auto current_time = std::chrono::steady_clock::now();
      const double delta_t =
        (std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time_stamp_))
          .count();

      switch (current_control_mode_)
      {
      case control_mode_type::POSITION:
        for (size_t i = 0; i < nr_dofs_; ++i)
        {
          auto new_velocity = (position_command[i] - joint_positions_[i]) / delta_t;
          new_velocity = new_velocity > MAX_VELOCITY ? MAX_VELOCITY : new_velocity;
          new_velocity = new_velocity < -MAX_VELOCITY ? -MAX_VELOCITY : new_velocity;

          joint_positions_[i] += new_velocity * delta_t;

          joint_accelerations_[i] = (new_velocity - joint_velocities_[i]) / delta_t;
          joint_velocities_[i] = new_velocity;
        }
        break;

      case control_mode_type::VELOCITY:
        for (size_t i = 0; i < nr_dofs_; ++i)
        {
          auto new_velocity = velocity_command[i];
          new_velocity = new_velocity > MAX_VELOCITY ? MAX_VELOCITY : new_velocity;
          new_velocity = new_velocity < MAX_VELOCITY ? -MAX_VELOCITY : new_velocity;

          joint_positions_[i] += new_velocity * delta_t;

          joint_accelerations_[i] = (new_velocity - joint_velocities_[i]) / delta_t;
          joint_velocities_[i] = new_velocity;
        }
        break;
      }
    }

    last_time_stamp_ = std::chrono::steady_clock::now();

    return true;
  }

  bool read_joint_states(
    std::vector<double> & position_state, std::vector<double> & velocity_state,
    std::vector<double> & acceleration_state)
  {
    position_state = joint_positions_;
    velocity_state = joint_velocities_;
    acceleration_state = joint_accelerations_;

    return true;
  }

  bool write_gpios(const std::vector<bool> & digital_outs)
  {
    digital_outs_ = digital_outs;
    return true;
  }

  bool read_gpio(std::vector<bool> & digital_ins, std::vector<bool> & digital_outs)
  {
    unsigned int seed = time(NULL);
    for (size_t i = 0; i < nr_dofs_; ++i)
    {
      digital_ins[i] = static_cast<bool>(rand_r(&seed) % 2);
    }

    digital_outs = digital_outs_;

    return true;
  }

  bool read_sensor_values(std::vector<double> & sensor_values)
  {
    for (size_t i = 0; i < sensor_values.size(); ++i)
    {
      unsigned int seed = time(NULL) + i;
      sensor_values[i] =
        static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX) * SENSOR_MAX;
    }

    return true;
  }

private:
  size_t nr_dofs_;
  control_mode_type current_control_mode_;

  std::vector<bool> digital_outs_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_accelerations_;

  std::chrono::steady_clock::time_point last_time_stamp_;

  bool movement_enabled_;
};
}  // namespace dr_denis_rrbot_comms

#endif  // CONTROLKO_HARDWARE_INTERFACE__DR_DENIS_RRBOT_COMMS_HPP_
