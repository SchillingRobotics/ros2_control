// Copyright (c) 2021, PickNik Inc.
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

/// \authors Andy Zelenak, Denis Stogl

#include "ruckig_joint_limiter/ruckig_joint_limiter.hpp"

#include <memory>
#include <string>
#include <vector>

#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "ruckig/input_parameter.hpp"
#include "ruckig/output_parameter.hpp"
#include "ruckig/ruckig.hpp"

namespace ruckig_joint_limiter
{
template <>
RuckigJointLimiter<joint_limits::JointLimits>::RuckigJointLimiter()
: joint_limits::JointLimiterInterface<joint_limits::JointLimits>()
{
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_init(/*const rclcpp::Duration & dt*/)
{
  // TODO(destogl): This should be used from parameter
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Initialize Ruckig
  ruckig_ = std::make_shared<ruckig::Ruckig<0>>(number_of_joints_, dt.seconds());
  ruckig_input_ = std::make_shared<ruckig::InputParameter<0>>(number_of_joints_);
  ruckig_output_ = std::make_shared<ruckig::OutputParameter<0>>(number_of_joints_);

  // Velocity mode works best for smoothing one waypoint at a time
  ruckig_input_->control_interface = ruckig::ControlInterface::Velocity;

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    if (joint_limits_[joint].has_jerk_limits)
    {
      ruckig_input_->max_jerk.at(joint) = joint_limits_[joint].max_acceleration;
    }
    if (joint_limits_[joint].has_acceleration_limits)
    {
      ruckig_input_->max_acceleration.at(joint) = joint_limits_[joint].max_acceleration;
    }
    if (joint_limits_[joint].has_velocity_limits)
    {
      ruckig_input_->max_velocity.at(joint) = joint_limits_[joint].max_velocity;
    }
  }

  return true;
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_configure(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states)
{
  // TODO(destogl): Direct association should be possible, we use Rucking with vectors
  // Initialize Ruckig with current_joint_states
  std::copy_n(
    current_joint_states.positions.begin(), number_of_joints_,
    ruckig_input_->current_position.begin());
  if (current_joint_states.velocities.size() == number_of_joints_)
  {
    std::copy_n(
      current_joint_states.velocities.begin(), number_of_joints_,
      ruckig_input_->current_velocity.begin());
  }
  else
  {
    auto vector_with_zeros = std::vector<double>(number_of_joints_, 0.0);
    std::copy_n(
      vector_with_zeros.begin(), number_of_joints_, ruckig_input_->current_velocity.begin());
  }
  if (current_joint_states.accelerations.size() == number_of_joints_)
  {
    std::copy_n(
      current_joint_states.accelerations.begin(), number_of_joints_,
      ruckig_input_->current_acceleration.begin());
  }
  else
  {
    auto vector_with_zeros = std::vector<double>(number_of_joints_, 0.0);
    std::copy_n(
      vector_with_zeros.begin(), number_of_joints_, ruckig_input_->current_acceleration.begin());
  }

  // Initialize output data
  ruckig_output_->new_position = ruckig_input_->current_position;
  ruckig_output_->new_velocity = ruckig_input_->current_velocity;
  ruckig_output_->new_acceleration = ruckig_input_->current_acceleration;

  return true;
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_states*/,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
  const rclcpp::Duration & /*dt*/)
{
  // We don't use current_joint_states. For stability, it is recommended to feed previous Ruckig
  // output back in as input for the next iteration. This assumes the robot tracks the command well.

  // Feed output from the previous timestep back as input
  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    // We do not have to this. This is done internally, by the library
    ruckig_input_->current_position.at(joint) = ruckig_output_->new_position.at(joint);
    ruckig_input_->current_velocity.at(joint) = ruckig_output_->new_velocity.at(joint);
    ruckig_input_->current_acceleration.at(joint) = ruckig_output_->new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input_->target_position.at(joint) = desired_joint_states.positions.at(joint);
    // TODO(destogl): in current use-case we use only velocity
    if (desired_joint_states.velocities.size() == number_of_joints_)
    {
      ruckig_input_->target_velocity.at(joint) = desired_joint_states.velocities.at(joint);
    }
    else
    {
      ruckig_input_->target_velocity.at(joint) = 0.0;
    }
    if (desired_joint_states.accelerations.size() == number_of_joints_)
    {
      ruckig_input_->target_acceleration.at(joint) = desired_joint_states.accelerations.at(joint);
    }
    else
    {
      ruckig_input_->target_acceleration.at(joint) = 0.0;
    }

    RCUTILS_LOG_INFO_NAMED(
      "ruckig_joint_limiter",
      "Desired position: %e \nDesired velocity: %e\n Desired acceleration: %e.",
      ruckig_input_->target_position.at(joint), ruckig_input_->target_velocity.at(joint),
      ruckig_input_->target_acceleration.at(joint));
  }

  ruckig::Result result = ruckig_->update(*ruckig_input_, *ruckig_output_);

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    RCUTILS_LOG_INFO_NAMED(
      "ruckig_joint_limiter", "New position: %e \nNew velocity: %e\nNew acceleration: %e.",
      ruckig_output_->new_position.at(joint), ruckig_output_->new_velocity.at(joint),
      ruckig_output_->new_acceleration.at(joint));
  }

  //   std::copy_n(ruckig_output_->new_position, number_of_joints_, desired_joint_states.positions);
  desired_joint_states.positions = ruckig_output_->new_position;
  //   std::copy_n(ruckig_output_->new_velocity, number_of_joints_,
  // desired_joint_states.velocities);
  desired_joint_states.velocities = ruckig_output_->new_velocity;
  //   std::copy_n(ruckig_output_->new_acceleration, number_of_joints_,
  // desired_joint_states.accelerations);
  desired_joint_states.accelerations = ruckig_output_->new_acceleration;

  return result == ruckig::Result::Finished;
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
