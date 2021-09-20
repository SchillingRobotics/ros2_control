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

#include <string>

namespace ruckig_joint_limiter
{
template <>
RuckigJointLimiter<limit_enforcement_plugins::JointLimits>::RuckigJointLimiter()
: limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>()
{
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<limit_enforcement_plugins::JointLimits>,
  limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>)
