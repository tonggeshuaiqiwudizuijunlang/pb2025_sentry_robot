#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <cstdlib>
#include <cmath>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "example_interfaces/msg/u_int8.hpp"


using namespace std::chrono_literals;

namespace simple_sentry_control
{

class SimpleSentryControlNode : public rclcpp::Node
{
public:
  SimpleSentryControlNode(const rclcpp::NodeOptions & options)
  : Node("simple_sentry_control_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Simple Sentry Control Node");

    // HP logic: <= low triggers retreat with highest priority, >= recover restarts forward route.
    this->declare_parameter("hp_low_threshold", 150);
    this->declare_parameter("hp_recover_threshold", 400);
    // Clamp: if recover threshold > referee max HP (e.g. 600), hp >= recover would never hold.
    this->declare_parameter("hp_max", 600);

    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("origin_yaw", 0.0);
    this->declare_parameter("origin_chassis_mode", 1); // 1 typically Free Mode, 3 for Spin

    this->declare_parameter("heal_x", 0.0);
    this->declare_parameter("heal_y", 0.0);
    this->declare_parameter("heal_yaw", 0.0);
    this->declare_parameter("heal_chassis_mode", 1);

    this->declare_parameter("point_a_x", 1.0);
    this->declare_parameter("point_a_y",  4.34);
    this->declare_parameter("point_a_yaw", 0.0);
    this->declare_parameter("point_a_chassis_mode", 3);

    this->declare_parameter("point_b_x", -1.9);
    this->declare_parameter("point_b_y", 5.2);
    this->declare_parameter("point_b_yaw", 0.0);
    this->declare_parameter("point_b_chassis_mode", 3);

    this->declare_parameter("point_c_x", -5.0);
    this->declare_parameter("point_c_y", 3.27);
    this->declare_parameter("point_c_yaw", 0.0);
    this->declare_parameter("point_c_chassis_mode", 3);

    this->declare_parameter("wait_for_match_start", false);

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    sub_robot_status_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", 10,
      [this](const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) { robot_status_ = msg; });

    sub_game_status_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
      "referee/game_status", 10,
      [this](const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) { game_status_ = msg; });

    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleSentryControlNode::controlLoop, this));

    // Transform listener for reaching point detection
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_chassis_mode_ = this->create_publisher<example_interfaces::msg::UInt8>("cmd_chassis_mode", 10);
  }

private:
  enum class Waypoint { HEAL, ORIGIN, A, B, C };
  enum class MissionPhase { FORWARD, RETREAT };

  struct Pose2D
  {
    double x;
    double y;
    double yaw;
  };

  void controlLoop()
  {
    if (!robot_status_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for robot_status topic...");
      return;
    }

    if (this->get_parameter("wait_for_match_start").as_bool()) {
      if (!game_status_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "等待比赛开始信号 (referee/game_status)...");
        return;
      }
      if (game_status_->game_progress != 4) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "当前比赛状态: %d (非 match_in_progress)，停留在原地。",
          static_cast<int>(game_status_->game_progress));
        if (nav_goal_active_) {
          cancelNavGoal();
        }
        return;
      }
    }

    const int hp = static_cast<int>(robot_status_->current_hp);
    const int hp_low = this->get_parameter("hp_low_threshold").as_int();
    const int hp_recover_raw = this->get_parameter("hp_recover_threshold").as_int();
    const int hp_max = std::max(1, this->get_parameter("hp_max").as_int());
    int hp_recover = std::min(hp_recover_raw, hp_max);
    if (hp_recover <= hp_low) {
      hp_recover = hp_low + 1;
    }

    if (hp_recover_raw > hp_max) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 30000,
        "hp_recover_threshold (%d) > hp_max (%d): effective recover HP = %d (否则满血也永远回不到前进)。",
        hp_recover_raw, hp_max, hp_recover);
    }

    if (hp <= hp_low) {
      if (phase_ != MissionPhase::RETREAT) {
        RCLCPP_WARN(this->get_logger(), "HP=%d <= %d, preempt current mission and retreat.", hp, hp_low);
        enterRetreatMode();
      }
      stepRetreat(hp, hp_recover);
    } else if (phase_ == MissionPhase::RETREAT) {
      if (hp >= hp_recover) {
        if (!nav_goal_active_ && current_waypoint_ == Waypoint::HEAL) {
          RCLCPP_INFO(this->get_logger(), "HP recovered to %d >= %d, restart HEAL->A->B->C.", hp, hp_recover);
          phase_ = MissionPhase::FORWARD;
          // current_waypoint_ remains HEAL; next stepForward sends HEAL -> A
        } else {
          stepRetreat(hp, hp_recover);
        }
      } else {
        stepRetreat(hp, hp_recover);
      }
    } else {
      if (pending_target_.has_value()) {
        const auto robot_pose = getRobotPose();
        if (robot_pose.has_value()) {
          const auto target_pose = waypointPose(pending_target_.value());
          double dist = std::hypot(robot_pose->x - target_pose.x, robot_pose->y - target_pose.y);
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "\033[93m[SentryControl] 当前目标: %d, 剩余距离: %.2f m\033[0m",
            static_cast<int>(pending_target_.value()), dist);
        }
      }
      stepForward();
    }

    // Single publish per tick: low HP / retreat always mode 1 (never 3 here).
    auto mode_msg = example_interfaces::msg::UInt8();
    if (hp <= hp_low || phase_ == MissionPhase::RETREAT) {
      mode_msg.data = 1;
    } else {
      const Waypoint active_wp = pending_target_.has_value() ? pending_target_.value() : current_waypoint_;
      switch (active_wp) {
        case Waypoint::HEAL:
          mode_msg.data = this->get_parameter("heal_chassis_mode").as_int();
          break;
        case Waypoint::ORIGIN:
          mode_msg.data = this->get_parameter("origin_chassis_mode").as_int();
          break;
        case Waypoint::A:
          mode_msg.data = this->get_parameter("point_a_chassis_mode").as_int();
          break;
        case Waypoint::B:
          mode_msg.data = this->get_parameter("point_b_chassis_mode").as_int();
          break;
        case Waypoint::C:
          mode_msg.data = this->get_parameter("point_c_chassis_mode").as_int();
          break;
        default:
          mode_msg.data = 1;
          break;
      }
    }
    pub_chassis_mode_->publish(mode_msg);
  }

  void stepForward()
  {
    if (nav_goal_active_) {
      if (pending_target_.has_value()) {
        const Pose2D target_pose = waypointPose(pending_target_.value());
        const auto robot_pose = getRobotPose();
        
        if (robot_pose.has_value()) {
          double dist = std::hypot(robot_pose->x - target_pose.x, robot_pose->y - target_pose.y);
          if (dist < 0.25) { 
            RCLCPP_INFO(this->get_logger(), "\033[92m[SentryControl] 接近目标点 (%f m)，切换到下一状态。\033[0m", dist);
            Waypoint reached_wp = pending_target_.value();
            cancelNavGoal(); 
            current_waypoint_ = reached_wp;
          }
        }
      }
      return;
    }

    switch (current_waypoint_) {
      case Waypoint::HEAL:
        sendGoalTo(Waypoint::A, "Forward: HEAL -> A");
        break;
      case Waypoint::ORIGIN:
        sendGoalTo(Waypoint::A, "Forward: O -> A");
        break;
      case Waypoint::A:
        sendGoalTo(Waypoint::B, "Forward: A -> B");
        break;
      case Waypoint::B:
        sendGoalTo(Waypoint::C, "Forward: B -> C");
        break;
      case Waypoint::C:
        // Already at C - stay here or you could potentially loop back to ORIGIN for patrol
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "[SentryControl] 已到达巡逻路径终点 C 点。");
        break;
    }
  }

  std::optional<Pose2D> getRobotPose()
  {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero);
      return Pose2D{t.transform.translation.x, t.transform.translation.y, 0.0};
    } catch (const tf2::TransformException & ex) {
      return std::nullopt;
    }
  }

  void enterRetreatMode()
  {
    phase_ = MissionPhase::RETREAT;
    cancelNavGoal();
  }

  void stepRetreat(int hp, int hp_recover)
  {
    if (nav_goal_active_) {
      return;
    }

    if (current_waypoint_ == Waypoint::HEAL) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[SentryControl] 已在 HEAL，当前 HP=%d，需 HP>=%d 才结束撤退并继续前进（见 hp_recover_threshold / hp_max）。",
        hp, hp_recover);
      return;
    }

    sendGoalTo(Waypoint::HEAL, "Retreat: direct to HEAL");
  }

  Pose2D waypointPose(const Waypoint wp) const
  {
    switch (wp) {
      case Waypoint::HEAL:
        return {
          this->get_parameter("heal_x").as_double(), this->get_parameter("heal_y").as_double(),
          this->get_parameter("heal_yaw").as_double()};
      case Waypoint::ORIGIN:
        return {
          this->get_parameter("origin_x").as_double(), this->get_parameter("origin_y").as_double(),
          this->get_parameter("origin_yaw").as_double()};
      case Waypoint::A:
        return {
          this->get_parameter("point_a_x").as_double(), this->get_parameter("point_a_y").as_double(),
          this->get_parameter("point_a_yaw").as_double()};
      case Waypoint::B:
        return {
          this->get_parameter("point_b_x").as_double(), this->get_parameter("point_b_y").as_double(),
          this->get_parameter("point_b_yaw").as_double()};
      case Waypoint::C:
        return {
          this->get_parameter("point_c_x").as_double(), this->get_parameter("point_c_y").as_double(),
          this->get_parameter("point_c_yaw").as_double()};
    }

    return {0.0, 0.0, 0.0};
  }

  void sendGoalTo(const Waypoint target, const std::string & reason)
  {
    const Pose2D pose = waypointPose(target);
    pending_target_ = target;
    sendNavGoal(pose.x, pose.y, pose.yaw, reason);
  }

  void sendNavGoal(double x, double y, double yaw, const std::string & reason)
  {
    if (nav_goal_active_ || !nav_client_->action_server_is_ready()) {
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 goal rejected");
        nav_goal_active_ = false;
        pending_target_.reset();
      }
    };

    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      nav_goal_active_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (pending_target_.has_value()) {
          current_waypoint_ = pending_target_.value();
        }
      }
      pending_target_.reset();
    };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    nav_goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), "%s | Goal sent: (%f, %f, %f)", reason.c_str(), x, y, yaw);
  }

  void cancelNavGoal()
  {
    if (nav_goal_active_) {
      nav_client_->async_cancel_all_goals();
      nav_goal_active_ = false;
    }
    pending_target_.reset();
  }

  MissionPhase phase_ = MissionPhase::FORWARD;
  Waypoint current_waypoint_ = Waypoint::ORIGIN;
  std::optional<Waypoint> pending_target_;

  pb_rm_interfaces::msg::RobotStatus::SharedPtr robot_status_;
  pb_rm_interfaces::msg::GameStatus::SharedPtr game_status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool nav_goal_active_ = false;

  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr sub_robot_status_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr sub_game_status_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr pub_chassis_mode_;
};

} // namespace simple_sentry_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_sentry_control::SimpleSentryControlNode)
