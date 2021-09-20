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

/// \authors Andy Zelenak

#include "ruckig_joint_limiter/ruckig_joint_limiter.hpp"

#include "limit_enforcement_plugins/joint_limits_rosparam.hpp"

#include <string>

namespace ruckig_joint_limiter
{
namespace
{
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3
}  // namespace

template <>
bool RuckigJointLimiter<limit_enforcement_plugins::JointLimits>::init(
  const std::vector<std::string> joint_names, const rclcpp::Node::SharedPtr & node,
  const std::string & robot_description_topic)
{
  auto num_dof_ = joint_names.size();
  joint_limits_.resize(num_dof_);
  bool result = true;

  // TODO(destogl): get limits from URDF
  // For now, limits are retrieved from the parameter server (typically yaml)

  for (auto i = 0ul; i < num_dof_; ++i)
  {
    if (!limit_enforcement_plugins::declare_parameters(joint_names[i], node))
    {
      RCLCPP_ERROR(
        node->get_logger(), "JointLimiter: Joint '%s': parameter declaration has failed",
        joint_names[i].c_str());
      result = false;
      break;
    }
    if (!limit_enforcement_plugins::get_limit_enforcement_plugins(joint_names[i], node, joint_limits_[i]))
    {
      RCLCPP_ERROR(
        node->get_logger(), "JointLimiter: Joint '%s': getting parameters has failed",
        joint_names[i].c_str());
      result = false;
      break;
    }
    RCLCPP_INFO(
      node->get_logger(), "Joint '%s':\n  %s", joint_names[i].c_str(),
      joint_limits_[i].to_string().c_str());
  }

  // ros2_control usually doesn't require a specific control rate a priori, but Ruckig does.
  // So, read it as a ROS parameter.
  double control_period_sec = 0;
  if (!node->get_parameter("control_period_seconds", control_period_sec))
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "The Ruckig joint limit plugin requires a `control_period_seconds` parameter");
    std::exit(EXIT_FAILURE);
  }

  // Initialize Ruckig
  ruckig_ = std::make_shared<ruckig::Ruckig<0>>(num_dof_, 0.01 /*timestep*/);
  ruckig_input_ = std::make_shared<ruckig::InputParameter<0>>(num_dof_);
  ruckig_output_ = std::make_shared<ruckig::OutputParameter<0>>(num_dof_);

  // Velocity mode works best for smoothing one waypoint at a time
  ruckig_input_->control_interface = ruckig::ControlInterface::Velocity;

  for (size_t jt = 0; jt < num_dof_; ++jt)
  {
    if (joint_limits_[jt].has_jerk_limits)
    {
      ruckig_input_->max_jerk.at(jt) = joint_limits_[jt].max_acceleration;
    }
    if (joint_limits_[jt].has_acceleration_limits)
    {
      ruckig_input_->max_acceleration.at(jt) = joint_limits_[jt].max_acceleration;
    }
    if (joint_limits_[jt].has_velocity_limits)
    {
      ruckig_input_->max_velocity.at(jt) = joint_limits_[jt].max_velocity;
    }
  }

  return result;
}

template <>
bool RuckigJointLimiter<limit_enforcement_plugins::JointLimits>::enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  // If this is the first call of the algorithm, initialize Ruckig with current_joint_states.
  // After that, we don't use current_joint_states. For stability, it is recommended to feed previous Ruckig output
  // back in as input for the next iteration. This assumes the robot tracks the command well.
  if (!received_initial_state_)
  {
    std::copy_n(current_joint_states.positions.begin(), num_dof_, ruckig_input_->current_position.begin());
    std::copy_n(current_joint_states.velocities.begin(), num_dof_, ruckig_input_->current_velocity.begin());
    std::copy_n(current_joint_states.accelerations.begin(), num_dof_, ruckig_input_->current_acceleration.begin());

    // Initialize output data struct
    ruckig_output_->new_position = ruckig_input_->current_position;
    ruckig_output_->new_velocity = ruckig_input_->current_velocity;
    ruckig_output_->new_acceleration = ruckig_input_->current_acceleration;
  }

  // Feed output from the previous timestep back as input
  for (size_t joint = 0; joint < num_dof_; ++joint)
  {
    ruckig_input_->current_position.at(joint) = ruckig_output_->new_position.at(joint);
    ruckig_input_->current_velocity.at(joint) = ruckig_output_->new_velocity.at(joint);
    ruckig_input_->current_acceleration.at(joint) = ruckig_output_->new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input_->target_velocity.at(joint) = desired_joint_states.velocities.at(joint);
    ruckig_input_->target_acceleration.at(joint) = desired_joint_states.accelerations.at(joint);
  }

  ruckig::Result result;
  if (ruckig_)
  {
    result = ruckig_->update(*ruckig_input_, *ruckig_output_);
  }
  else
  {
    return false;
  }

  return result == ruckig::Result::Finished;
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<limit_enforcement_plugins::JointLimits>,
  limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>)
