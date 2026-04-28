// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
// ros2
#include <tf2_ros/buffer.h>
#include <angles/angles.h>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"
#include "armor_solver/gimbal_mpc_controller.hpp"

namespace fyt::auto_aim {

// ─────────────────────────────────────────────────────────────────────────────
// 4-state Auto-Aim FSM  (ported from wust_vision VeryAimer / AutoAimFsmController)
//
// State transitions driven by |v_yaw| (rad/s):
//
//   SINGLE ──(high spin)──► WHOLE_ARMOR ──(higher)──► WHOLE_PAIR ──(very high)──► CENTER
//          ◄──(slow down)──              ◄───────────            ◄───────────────
// ─────────────────────────────────────────────────────────────────────────────
enum class AutoAimFsm {
  AIM_SINGLE_ARMOR,      ///< Slow spin  – lock onto one armor
  AIM_WHOLE_CAR_ARMOR,   ///< Medium spin – track whole car, pick nearest armor
  AIM_WHOLE_CAR_PAIR,    ///< Faster spin – switch between armor pair
  AIM_WHOLE_CAR_CENTER,  ///< Very fast   – aim at car geometric center
};

inline const char* autoAimFsmToString(AutoAimFsm s) {
  switch (s) {
    case AutoAimFsm::AIM_SINGLE_ARMOR:     return "SINGLE";
    case AutoAimFsm::AIM_WHOLE_CAR_ARMOR:  return "WHOLE_ARMOR";
    case AutoAimFsm::AIM_WHOLE_CAR_PAIR:   return "WHOLE_PAIR";
    case AutoAimFsm::AIM_WHOLE_CAR_CENTER: return "CENTER";
  }
  return "UNKNOWN";
}

/**
 * Four-state FSM controller.
 * Call update() every time a new tracker state becomes available.
 * All threshold fields are public so they can be set from ROS2 parameters.
 */
class AutoAimFsmController {
public:
  AutoAimFsm state{AutoAimFsm::AIM_SINGLE_ARMOR};

  // v_yaw thresholds [rad/s]
  double single_whole_up   = 3.0;
  double single_whole_down = 2.0;
  double whole_pair_up     = 5.0;
  double whole_pair_down   = 4.0;
  double pair_center_up    = 8.0;
  double pair_center_down  = 6.0;
  int    transfer_thresh   = 3;

  /**
   * @param v_yaw        Current tracked angular velocity [rad/s]
   * @param is_same_target  True when we are continuously tracking the SAME target
   *                        (i.e., target did NOT jump). False when tracker just
   *                        switched to a new robot → reset FSM to SINGLE.
   */
  void update(double v_yaw, bool is_same_target) {
    if (!is_same_target) {
      state = AutoAimFsm::AIM_SINGLE_ARMOR;
      cnt_ = 0;
      return;
    }
    const double av = std::abs(v_yaw);
    switch (state) {
      case AutoAimFsm::AIM_SINGLE_ARMOR:
        cnt_ = (av > single_whole_up) ? cnt_ + 1 : 0;
        if (cnt_ > transfer_thresh) {
          state = AutoAimFsm::AIM_WHOLE_CAR_ARMOR;
          cnt_ = 0;
        }
        break;

      case AutoAimFsm::AIM_WHOLE_CAR_ARMOR:
        if (av > whole_pair_up)          ++cnt_;
        else if (av < single_whole_down) --cnt_;
        else                              cnt_ = 0;
        if (std::abs(cnt_) > transfer_thresh) {
          state = (cnt_ > 0) ? AutoAimFsm::AIM_WHOLE_CAR_PAIR
                              : AutoAimFsm::AIM_SINGLE_ARMOR;
          cnt_ = 0;
        }
        break;

      case AutoAimFsm::AIM_WHOLE_CAR_PAIR:
        if (av > pair_center_up)        ++cnt_;
        else if (av < whole_pair_down)  --cnt_;
        else                             cnt_ = 0;
        if (std::abs(cnt_) > transfer_thresh) {
          state = (cnt_ > 0) ? AutoAimFsm::AIM_WHOLE_CAR_CENTER
                              : AutoAimFsm::AIM_WHOLE_CAR_ARMOR;
          cnt_ = 0;
        }
        break;

      case AutoAimFsm::AIM_WHOLE_CAR_CENTER:
        cnt_ = (av < pair_center_down) ? cnt_ + 1 : 0;
        if (cnt_ > transfer_thresh) {
          state = AutoAimFsm::AIM_WHOLE_CAR_PAIR;
          cnt_ = 0;
        }
        break;
    }
  }

private:
  int cnt_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Solver
// ─────────────────────────────────────────────────────────────────────────────
class Solver {
public:
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  ~Solver() = default;

  /**
   * Compute GimbalCmd for the current tracker target.
   * @param fsm  Current AutoAimFsm state (drives armor selection strategy).
   * @throws tf2::TransformException when TF is unavailable.
   */
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time               &current_time,
                                      std::shared_ptr<tf2_ros::Buffer>  tf2_buffer_,
                                      AutoAimFsm fsm = AutoAimFsm::AIM_SINGLE_ARMOR);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

  void resetMpc() { reset_mpc_ = true; }

  std::vector<std::pair<double, double>> getTrajectory() const noexcept;

private:
  // ── Geometry helpers ─────────────────────────────────────────────────────

  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 double yaw,
                                                 double r1, double r2,
                                                 double d_zc, double d_za,
                                                 size_t armors_num) const noexcept;

  // Legacy 2-state selection (side_angle_ based)
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      double target_yaw, double target_v_yaw,
                      size_t armors_num) const noexcept;

  /**
   * 4-state FSM armor selection.
   * Ported from wust_vision VeryAimer::Impl::selectArmor().
   *
   * delta_angle[i] = normalize(armor_yaw[i] - center_yaw)
   *
   * SINGLE:       lock onto the armor nearest δ=0, keep lock_id stable
   * WHOLE_ARMOR:  pick armor inside coming_angle, use leaving_angle hysteresis
   * WHOLE_PAIR:   pick from alternate pair (indices 1,3 or 0,2)
   * CENTER:       no individual armor, caller uses target center
   */
  int selectArmorFsm(const std::vector<Eigen::Vector3d> &armor_positions,
                     const Eigen::Vector3d &target_center,
                     double target_yaw, double target_v_yaw,
                     size_t armors_num,
                     AutoAimFsm fsm) const noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       std::array<double, 3> rpy,
                       double &yaw, double &pitch) const noexcept;

  bool isOnTarget(const rm_interfaces::msg::Target &target,
                  double cur_yaw, double cur_pitch,
                  double target_yaw, double target_pitch,
                  double distance, double diff_yaw,
                  AutoAimFsm fsm) const noexcept;

  // ── Components ───────────────────────────────────────────────────────────

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator>     manual_compensator_;

  // TinyMPC-based gimbal trajectory smoother
  // Enabled at runtime via ROS2 param "solver.enable_mpc"
  std::unique_ptr<GimbalMpcController> mpc_controller_;
  bool enable_mpc_ = false;
  bool reset_mpc_ = true;

  // ── Persistent gimbal state (MPC warm-start) ─────────────────────────────
  double prev_yaw_cmd_   = 0.0;
  double prev_pitch_cmd_ = 0.0;
  double prev_yaw_vel_   = 0.0;
  double prev_pitch_vel_ = 0.0;
  rclcpp::Time prev_solve_time_{0, 0, RCL_ROS_TIME};  ///< Timestamp of the last solve() call

  // ── Parameters ───────────────────────────────────────────────────────────
  std::array<double, 3> rpy_{};

  double prediction_delay_      = 0.0;
  double controller_delay_      = 0.0;
  double yaw_prediction_scale_  = 0.6;

  double shooting_range_w_      = 0.13;
  double shooting_range_h_      = 0.13;

  double max_tracking_v_yaw_    = 6.0;
  int    overflow_count_        = 0;
  int    transfer_thresh_       = 5;

  double side_angle_            = 15.0;
  double min_switching_v_yaw_   = 1.0;

  // VeryAimer-style FSM armor selection
  double coming_angle_deg_  = 60.0;   ///< [deg] armor "incoming" cone half-angle
  double leaving_angle_deg_ = 45.0;   ///< [deg] armor "leaving" threshold

  // Sticky armor id to avoid flickering between adjacent armors
  mutable int locked_armor_id_ = -1;

  std::weak_ptr<rclcpp::Node> node_;
};

}  // namespace fyt::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_
