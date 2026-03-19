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

#include "pb2025_sentry_behavior/plugins/action/pub_chassis_mode.hpp"

namespace pb2025_sentry_behavior
{

PublishChassisModeAction::PublishChassisModeAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishChassisModeAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<uint8_t>(
      "chassis_mode", 1,
      "Chassis mode: 0=disabled, 1=free, 2=follow, 3=spin (小陀螺)"),
  });
}

bool PublishChassisModeAction::setMessage(example_interfaces::msg::UInt8 & msg)
{
  uint8_t mode = 1;  // 默认自由模式
  getInput("chassis_mode", mode);
  msg.data = mode;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishChassisModeAction, "PublishChassisMode");
