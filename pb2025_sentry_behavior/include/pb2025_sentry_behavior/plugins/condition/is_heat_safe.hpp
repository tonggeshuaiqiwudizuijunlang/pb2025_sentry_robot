// Copyright 2025 SMBU-PolarBear-Robotics-Team
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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HEAT_SAFE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HEAT_SAFE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that checks whether the shooter heat is within
 * the safe range to allow firing.
 *
 * Returns SUCCESS when current heat < heat_limit * (heat_ratio / 100).
 * Returns FAILURE when overheated — the behavior tree should stop firing.
 *
 * Usage in XML:
 *   <IsHeatSafe heat_ratio="80" key_port="{@referee_robotStatus}"/>
 */
class IsHeatSafeCondition : public BT::SimpleConditionNode
{
public:
  IsHeatSafeCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Checks shooter heat against the configured safe ratio.
   *
   * Uses shooter_17mm_1_barrel_heat and shooter_barrel_heat_limit from
   * RobotStatus. If heat_limit is 0 (no data yet), returns SUCCESS to
   * avoid blocking before the referee system reports data.
   */
  BT::NodeStatus checkHeatSafe();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsHeatSafeCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HEAT_SAFE_HPP_
