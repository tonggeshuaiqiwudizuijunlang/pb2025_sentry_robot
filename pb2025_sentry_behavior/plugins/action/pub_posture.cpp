// Copyright 2025 Lihan Chen
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

#include "pb2025_sentry_behavior/plugins/action/pub_posture.hpp"

namespace pb2025_sentry_behavior
{

PublishPostureAction::PublishPostureAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishPostureAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<uint8_t>("posture", 0, "Robot posture (0:Move, 1:Attack, 2:Defend)"),
  });
}

bool PublishPostureAction::setMessage(example_interfaces::msg::UInt8 & msg)
{
  uint8_t posture = 0;
  getInput("posture", posture);

  msg.data = posture;

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishPostureAction, "PublishPosture");
