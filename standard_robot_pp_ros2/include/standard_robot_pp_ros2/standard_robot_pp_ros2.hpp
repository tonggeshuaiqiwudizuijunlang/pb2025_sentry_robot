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

#ifndef STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
#define STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>

#include "rm_interfaces/msg/target.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_state_info.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "standard_robot_pp_ros2/robot_info.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace standard_robot_pp_ros2
{
class StandardRobotPpRos2Node : public rclcpp::Node
{
public:
  explicit StandardRobotPpRos2Node(const rclcpp::NodeOptions & options);

  ~StandardRobotPpRos2Node() override;

private:
  bool is_usb_ok_;
  bool debug_;
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  bool record_rosbag_;
  bool set_detector_color_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;

  // Publish
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr yq_debug_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_gimbal_joint_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr cmd_tracking_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_posture_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_chassis_mode_sub_;

  RobotModels robot_models_;
  std::unordered_map<std::string, rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr>
    debug_pub_map_;

  SendRobotCmdData send_robot_cmd_data_;

  void getParams();
  void createPublisher();
  void createSubscription();
  void createNewDebugPublisher(const std::string & name);
  void receiveData();
  void sendData();
  void serialPortProtect();

  void publishDebugData(ReceiveDebugData & data);
  void publishImuData(ReceiveImuData & data);
  void publishRobotInfo(ReceiveRobotInfoData & data);
  void publishEventData(ReceiveEventData & data);
  void publishAllRobotHp(ReceiveAllRobotHpData & data);
  void publishGameStatus(ReceiveGameStatusData & data);
  void publishRobotMotion(ReceiveRobotMotionData & data);
  void publishGroundRobotPosition(ReceiveGroundRobotPosition & data);
  void publishRfidStatus(ReceiveRfidStatus & data);
  void publishRobotStatus(ReceiveRobotStatus & data);
  void publishJointState(ReceiveJointState & data);
  void publishBuff(ReceiveBuff & data);
  void Decode_YQ_Data(Receive_YQ_Serial_State & imu_data);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
  void visionTargetCallback(const rm_interfaces::msg::Target::SharedPtr msg);
  void cmdGimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
  void cmdPostureCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
  void cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg);
  void cmdChassisModeCallback(const example_interfaces::msg::UInt8::SharedPtr msg);

  void setParam(const rclcpp::Parameter & param);
  bool getDetectColor(uint8_t robot_id, uint8_t & color);
  bool callTriggerService(const std::string & service_name);

  // Param client to set detect_color
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;
  std::string detector_node_name_;

  uint8_t previous_game_progress_ = 0;

  float last_hp_;
  float last_gimbal_pitch_odom_joint_, last_gimbal_yaw_odom_joint_;

  // 均值滤波相关
  static constexpr size_t FILTER_WINDOW_SIZE = 3;  // 滑动窗口大小
  std::vector<float> pitch_buffer_;  // pitch 历史值缓冲区
  std::vector<float> yaw_buffer_;    // yaw 历史值缓冲区
  bool last_tracking_ = false;       // 上一次的 tracking 状态

  std::mutex cmd_mutex_;  // 保护 send_robot_cmd_data_

  // ── 热量安全 flag ──────────────────────────────────────────────────
  // 由 Decode_YQ_Data 每帧更新。
  std::atomic<bool> heat_safe_{true};  // true = 热量安全可开火
  int heat_ratio_{80};  // 安全热量百分比阈值（heat < limit × heat_ratio_/100 = safe）
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
