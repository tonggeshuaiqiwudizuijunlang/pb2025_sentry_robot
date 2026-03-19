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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_PATROL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_PATROL_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb2025_sentry_behavior
{

/**
 * @brief Publishes a patrol start/stop command to pb2025_nav2_patrol.
 *
 * Ports:
 *   enable (bool): true = start patrol, false = stop patrol
 *   topic_name (string): target ROS topic (default: /start_patrol)
 */
class PublishPatrolAction
: public BT::RosTopicPubStatefulActionNode<std_msgs::msg::Bool>
{
public:
  PublishPatrolAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(std_msgs::msg::Bool & msg) override;

  // 停止时自动发 false
  bool setHaltMessage(std_msgs::msg::Bool & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_PATROL_HPP_
