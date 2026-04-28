#include <cmath>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "example_interfaces/msg/u_int8.hpp"


using namespace std::chrono_literals;

namespace simple_sentry_control
{

class SimpleSentryControlTwoPointNode : public rclcpp::Node
{
public:
  explicit SimpleSentryControlTwoPointNode(const rclcpp::NodeOptions & options)
  : Node("simple_sentry_control_two_point_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Simple Sentry Control Two-Point Node");

    this->declare_parameter("hp_low_threshold", 150);
    this->declare_parameter("hp_recover_threshold", 400);

    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("origin_yaw", 0.0);

    this->declare_parameter("point_a_x", 1.0);
    this->declare_parameter("point_a_y", 0.0);
    this->declare_parameter("point_a_yaw", 0.0);
    this->declare_parameter("chassis_mode", 3);

    nav_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    sub_robot_status_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", 10,
      [this](const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) { robot_status_ = msg; });

    sub_game_status_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
      "referee/game_status", 10,
      [this](const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) { game_status_ = msg; });

    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleSentryControlTwoPointNode::controlLoop, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_chassis_mode_ = this->create_publisher<example_interfaces::msg::UInt8>("cmd_chassis_mode", 10);
  }

private:
  enum class Waypoint { ORIGIN, A };
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
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Waiting for robot_status topic...");
      return;
    }

    // [Safety Check] Only allow execution if game_status is "Match in Progress" (4)
    if (!game_status_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "\033[93m等待比赛开始信号 (game_status)... \033[0m");
      return;
    }

    if (game_status_->game_progress != 4) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "\033[91m当前比赛状态: %d (非 match_in_progress)，停留在原地。\033[0m", 
        static_cast<int>(game_status_->game_progress));
      
      if (nav_goal_active_) {
        cancelNavGoal();
      }
      return;
    }


    const int hp = static_cast<int>(robot_status_->current_hp);
    const int hp_low = this->get_parameter("hp_low_threshold").as_int();
    const int hp_recover = this->get_parameter("hp_recover_threshold").as_int();

    if (hp <= hp_low) {
      if (phase_ != MissionPhase::RETREAT) {
        RCLCPP_WARN(this->get_logger(), "HP=%d <= %d, preempt and return to origin.", hp, hp_low);
        enterRetreatMode();
      }
      stepRetreat();
      return;
    }

    if (phase_ == MissionPhase::RETREAT) {
      if (hp >= hp_recover && !nav_goal_active_ && current_waypoint_ == Waypoint::ORIGIN) {
        RCLCPP_INFO(
          this->get_logger(), "HP recovered to %d >= %d, restart ORIGIN -> A.", hp, hp_recover);
        phase_ = MissionPhase::FORWARD;
      } else {
        stepRetreat();
      }
      return;
    }

    stepForward();

    // Publish chassis mode based on game status
    auto mode_msg = example_interfaces::msg::UInt8();
    if (game_status_ && game_status_->game_progress == 4) {
      mode_msg.data = this->get_parameter("chassis_mode").as_int();
    } else {
      mode_msg.data = 1;  // Free mode (自由)
    }
    pub_chassis_mode_->publish(mode_msg);
  }

  void stepForward()
  {
    if (nav_goal_active_) {
      if (pending_target_.has_value()) {
        const Pose2D target = waypointPose(pending_target_.value());
        const auto robot = getRobotPose();
        if (robot.has_value()) {
          double dist = std::hypot(robot->x - target.x, robot->y - target.y);
          if (dist < 0.25) {
            RCLCPP_INFO(this->get_logger(), "[TwoPoint] 接近目标点 (%f m)，切换。", dist);
            Waypoint reached = pending_target_.value();
            cancelNavGoal();
            current_waypoint_ = reached;
          }
        }
      }
      return;
    }

    if (current_waypoint_ == Waypoint::ORIGIN) {
      sendGoalTo(Waypoint::A, "Forward: O -> A");
      return;
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

  void stepRetreat()
  {
    if (nav_goal_active_) {
      return;
    }

    if (current_waypoint_ != Waypoint::ORIGIN) {
      sendGoalTo(Waypoint::ORIGIN, "Retreat: return to origin");
    }
  }

  Pose2D waypointPose(const Waypoint wp) const
  {
    if (wp == Waypoint::ORIGIN) {
      return {
        this->get_parameter("origin_x").as_double(), this->get_parameter("origin_y").as_double(),
        this->get_parameter("origin_yaw").as_double()};
    }

    return {
      this->get_parameter("point_a_x").as_double(), this->get_parameter("point_a_y").as_double(),
      this->get_parameter("point_a_yaw").as_double()};
  }

  void sendGoalTo(const Waypoint target, const std::string & reason)
  {
    const Pose2D pose = waypointPose(target);
    pending_target_ = target;
    sendNavGoal(pose.x, pose.y, pose.yaw, reason);
  }

  void sendNavGoal(double x, double y, double yaw, const std::string & reason)
  {
    if (nav_goal_active_) {
      return;
    }

    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
      pending_target_.reset();
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

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &
      goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected by server");
          nav_goal_active_ = false;
          pending_target_.reset();
        } else {
          RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted by server, waiting for result");
        }
      };

    send_goal_options.result_callback =
      [this](
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
      result) {
        nav_goal_active_ = false;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Nav2 goal succeeded!");
            if (pending_target_.has_value()) {
              current_waypoint_ = pending_target_.value();
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Nav2 goal was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        pending_target_.reset();
      };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    nav_goal_active_ = true;
    RCLCPP_INFO(
      this->get_logger(), "%s | Sent Nav2 goal to (%f, %f, yaw=%f)", reason.c_str(), x, y,
      yaw);
  }

  void cancelNavGoal()
  {
    if (nav_goal_active_) {
      nav_client_->async_cancel_all_goals();
      nav_goal_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Cancelled Nav2 goals");
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
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr sub_robot_status_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr sub_game_status_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool nav_goal_active_ = false;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr pub_chassis_mode_;
};

}  // namespace simple_sentry_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_sentry_control::SimpleSentryControlTwoPointNode)
