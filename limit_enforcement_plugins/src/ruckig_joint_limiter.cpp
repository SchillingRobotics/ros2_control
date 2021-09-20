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

#include <string>

namespace ruckig_joint_limiter
{
template <>
bool RuckigJointLimiter<limit_enforcement_plugins::JointLimits>::init(
  const std::vector<std::string> joint_names, const rclcpp::Node::SharedPtr & node,
  const std::string & robot_description_topic)
{
  return true;
}

template <>
bool RuckigJointLimiter<limit_enforcement_plugins::JointLimits>::enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  return true;
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<limit_enforcement_plugins::JointLimits>,
  limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>)
