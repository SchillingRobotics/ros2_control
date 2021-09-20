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

/// \author Andy Zelenak, Denis Stogl

#ifndef RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
#define RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_

#include "limit_enforcement_plugins/joint_limiter_interface.hpp"
#include "limit_enforcement_plugins/joint_limits.hpp"

namespace ruckig_joint_limiter
{
template <typename LimitsType>
class RuckigJointLimiter : public limit_enforcement_plugins::JointLimiterInterface<limit_enforcement_plugins::JointLimits>
{
public:
  JOINT_LIMITS_PUBLIC RuckigJointLimiter();
};

}  // namespace ruckig_joint_limiter

#endif  // RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
