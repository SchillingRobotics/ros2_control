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

/// \author Denis Stogl

// Simple joint limiter using a clamping approach.
// May violate acceleration and jerk limits.
// May not continue on the intended path.

#ifndef JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
#define JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_

#include <string>

#include "limit_enforcement_plugins/joint_limiter_interface.hpp"
#include "limit_enforcement_plugins/joint_limits.hpp"

namespace limit_enforcement_plugins
{
template <typename LimitsType>
class SimpleJointLimiter : public JointLimiterInterface<JointLimits>
{
public:
  JOINT_LIMITS_PUBLIC bool init(
    const std::vector<std::string> joint_names, const rclcpp::Node::SharedPtr & node,
    const std::string & robot_description_topic = "/robot_description") override;

  JOINT_LIMITS_PUBLIC bool enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) override;

private:
  std::vector<LimitsType> joint_limits_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace limit_enforcement_plugins

#endif  // JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
