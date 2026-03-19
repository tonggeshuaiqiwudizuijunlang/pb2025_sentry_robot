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

#include "pb2025_sentry_behavior/plugins/action/pub_shoot.hpp"

namespace pb2025_sentry_behavior
{

PublishShootAction::PublishShootAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishShootAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("fire", 0, "Fire command: 0=stop, 1=single shot, 2=auto fire"),
  });
}

bool PublishShootAction::setMessage(example_interfaces::msg::UInt8 & msg)
{
  int fire = 0;
  getInput("fire", fire);
  msg.data = static_cast<uint8_t>(fire);
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishShootAction, "PublishShoot");
