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

/// \author Andy Zelenak

#ifndef RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
#define RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_

#include "limit_enforcement_plugins/joint_limiter_interface.hpp"
#include "limit_enforcement_plugins/joint_limits.hpp"

#include "ruckig/ruckig.hpp"

namespace ruckig_joint_limiter
{
template <typename LimitsType>
class RuckigJointLimiter : public limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>
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
  // Store joint limit data
  std::vector<LimitsType> joint_limits_;
  size_t num_dof_;
  bool received_initial_state_ = false;
  // Ruckig algorithm
  std::shared_ptr<ruckig::Ruckig<0>> ruckig_;
  std::shared_ptr<ruckig::InputParameter<0>> ruckig_input_;
  std::shared_ptr<ruckig::OutputParameter<0>> ruckig_output_;
};

}  // namespace ruckig_joint_limiter

#endif  // RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
