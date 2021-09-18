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

/// \authors Nathan Brooks, Denis Stogl

#include "rucking_joint_limiter/rucking_joint_limiter.hpp"

#include <string>

namespace rucking_joint_limiter
{
template <>
RuckingJointLimiter<joint_limits::JointLimits>::RuckingJointLimiter()
: joint_limits::JointLimiterInterface<joint_limits::JointLimits>()
{
}

template <>
bool RuckingJointLimiter<joint_limits::JointLimits>::on_init()
{
  return true;
}

template <>
bool RuckingJointLimiter<joint_limits::JointLimits>::on_configure()
{
  return true;
}

template <>
bool RuckingJointLimiter<joint_limits::JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  desired_joint_states = current_joint_states;

  return true;
}

}  // namespace rucking_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rucking_joint_limiter::RuckingJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
