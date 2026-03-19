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

#include "pb2025_sentry_behavior/plugins/action/pub_patrol.hpp"

namespace pb2025_sentry_behavior
{

PublishPatrolAction::PublishPatrolAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishPatrolAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<bool>("enable", true, "true = start patrol, false = stop patrol"),
  });
}

bool PublishPatrolAction::setMessage(std_msgs::msg::Bool & msg)
{
  bool enable = true;
  getInput("enable", enable);
  msg.data = enable;
  return true;
}

bool PublishPatrolAction::setHaltMessage(std_msgs::msg::Bool & msg)
{
  // 行为树节点被停止时，自动发送 false 停止巡逻
  msg.data = false;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishPatrolAction, "PublishPatrol");
