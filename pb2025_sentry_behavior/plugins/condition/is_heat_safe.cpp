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

#include "pb2025_sentry_behavior/plugins/condition/is_heat_safe.hpp"

namespace pb2025_sentry_behavior
{

IsHeatSafeCondition::IsHeatSafeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHeatSafeCondition::checkHeatSafe, this), config)
{
}

BT::NodeStatus IsHeatSafeCondition::checkHeatSafe()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    // 裁判系统数据尚未到达，允许开火避免卡死
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock().make_shared(), 5000,
      "IsHeatSafeCondition: RobotStatus not available yet, allowing fire by default");
    return BT::NodeStatus::SUCCESS;
  }

  int heat_ratio = 80;
  getInput("heat_ratio", heat_ratio);

  const uint16_t heat_limit = msg->shooter_barrel_heat_limit;
  const uint16_t current_heat = msg->shooter_17mm_1_barrel_heat;

  // 若热量上限为0（数据未初始化），默认允许
  if (heat_limit == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  // 计算安全热量阈值：热量 < 上限 × (heat_ratio / 100)
  const uint16_t safe_threshold = static_cast<uint16_t>(heat_limit * heat_ratio / 100);
  const bool is_safe = (current_heat < safe_threshold);

  if (!is_safe) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock().make_shared(), 1000,
      "IsHeatSafeCondition: Overheat! heat=%d / limit=%d (safe_threshold=%d). Blocking fire.",
      current_heat, heat_limit, safe_threshold);
  }

  return is_safe ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsHeatSafeCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>(
      "heat_ratio", 80,
      "Max heat ratio (%) of shooter_barrel_heat_limit to allow firing. "
      "Default 80 means fire only when heat < 80% of limit.")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsHeatSafeCondition>("IsHeatSafe");
}
