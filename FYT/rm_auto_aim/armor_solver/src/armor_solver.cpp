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

#include "armor_solver/armor_solver.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// std
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {

Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_    = node->declare_parameter("solver.shooting_range_width",  0.13);
  shooting_range_h_    = node->declare_parameter("solver.shooting_range_height", 0.13);
  max_tracking_v_yaw_  = node->declare_parameter("solver.max_tracking_v_yaw",    6.0);
  prediction_delay_    = node->declare_parameter("solver.prediction_delay",       0.0);
  controller_delay_    = node->declare_parameter("solver.controller_delay",       0.0);
  yaw_prediction_scale_= node->declare_parameter("solver.yaw_prediction_scale",  0.6);
  side_angle_          = node->declare_parameter("solver.side_angle",             15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw",   1.0);

  // VeryAimer-style coming/leaving angle for FSM armor selection
  coming_angle_deg_  = node->declare_parameter("solver.coming_angle_deg",  60.0);
  leaving_angle_deg_ = node->declare_parameter("solver.leaving_angle_deg", 45.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity        = node->declare_parameter("solver.bullet_speed",   20.0);
  trajectory_compensator_->gravity         = node->declare_parameter("solver.gravity",         9.8);
  trajectory_compensator_->resistance      = node->declare_parameter("solver.resistance",       0.001);

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if (!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state            = State::TRACKING_ARMOR;
  overflow_count_  = 0;
  transfer_thresh_ = 5;

  // ── TinyMPC initialization ──────────────────────────────────────────────
  enable_mpc_ = node->declare_parameter("solver.enable_mpc", false);
  if (enable_mpc_) {
    MpcParams mp;
    mp.horizon       = node->declare_parameter("solver.mpc_horizon",        20);
    mp.dt            = node->declare_parameter("solver.mpc_dt",             0.005);
    mp.Q_pos         = node->declare_parameter("solver.mpc_q_pos",          100.0);
    mp.Q_vel         = node->declare_parameter("solver.mpc_q_vel",          1.0);
    mp.R_acc         = node->declare_parameter("solver.mpc_r_acc",          0.01);
    mp.rho           = node->declare_parameter("solver.mpc_rho",            1.0);
    mp.max_acc_yaw   = node->declare_parameter("solver.mpc_max_acc_yaw",    60.0);
    mp.max_acc_pitch = node->declare_parameter("solver.mpc_max_acc_pitch",  40.0);
    mpc_controller_ = std::make_unique<GimbalMpcController>(mp);
    FYT_INFO("armor_solver", "TinyMPC trajectory smoother ENABLED (horizon={}, dt={}s)",
      mp.horizon, mp.dt);
  }
  // ────────────────────────────────────────────────────────────────────────

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time               &current_time,
                                            std::shared_ptr<tf2_ros::Buffer>  tf2_buffer_,
                                            AutoAimFsm fsm) {
  // Get newest parameters
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_  = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_    = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_    = node->get_parameter("solver.controller_delay").as_double();
    yaw_prediction_scale_= node->get_parameter("solver.yaw_prediction_scale").as_double();
    side_angle_          = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    coming_angle_deg_    = node->get_parameter("solver.coming_angle_deg").as_double();
    leaving_angle_deg_   = node->get_parameter("solver.leaving_angle_deg").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "vision_camera_link", tf2::TimePointZero); //camera_link gimbal_link
    auto msg_q = gimbal_tf.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Predict target position iteratively (flying time + prediction delay)
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d initial_position = target_position;
  double target_yaw = target.yaw;
  double time_diff = (current_time - rclcpp::Time(target.header.stamp)).seconds();
  double flying_time = trajectory_compensator_->getFlyingTime(initial_position);
  double dt = time_diff + flying_time + prediction_delay_;

  for (int i = 0; i < 10; ++i) {
    dt = time_diff + flying_time + prediction_delay_;
    target_position.x() = initial_position.x() + dt * target.velocity.x;
    target_position.y() = initial_position.y() + dt * target.velocity.y;
    target_position.z() = initial_position.z() + dt * target.velocity.z;

    double next_flying_time = trajectory_compensator_->getFlyingTime(target_position);
    if (std::abs(next_flying_time - flying_time) < 1e-3) {
      flying_time = next_flying_time;
      break;
    }
    flying_time = next_flying_time;
  }
  
  target_yaw += dt * yaw_prediction_scale_ * target.v_yaw;

  // ── Armor selection ────────────────────────────────────────────────────────
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw,
    target.radius_1, target.radius_2,
    target.d_zc, target.d_za, target.armors_num);

  int idx;
  // AIM_WHOLE_CAR_CENTER: aim at the car center (no individual armor)
  if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
    // closest armor to camera gives range info; actual aim is center
    idx = 0;
  } else {
    // Use 4-state FSM selection for all other states
    idx = selectArmorFsm(armor_positions, target_position, target_yaw,
                         target.v_yaw, target.armors_num, fsm);
  }

  Eigen::Vector3d chosen_armor_position = (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER)
    ? target_position  // aim at center
    : armor_positions.at(idx);

  if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // Initialize gimbal_cmd
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header       = target.header;
  gimbal_cmd.distance     = distance;
  gimbal_cmd.fire_advice  = false;

  // ── Legacy 2-state tracking switch (still drives overflow_count_) ─────────
  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) overflow_count_++;
      else                                               overflow_count_ = 0;
      if (overflow_count_ > transfer_thresh_)            state = TRACKING_CENTER;

      if (controller_delay_ != 0) {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * yaw_prediction_scale_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position, target_yaw,
                                            target.radius_1, target.radius_2,
                                            target.d_zc, target.d_za,
                                            target.armors_num);
        chosen_armor_position = armor_positions.at(idx);
        gimbal_cmd.distance   = chosen_armor_position.norm();
        if (chosen_armor_position.norm() < 0.1)
          throw std::runtime_error("No valid armor to shoot");
        calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) overflow_count_++;
      else                                               overflow_count_ = 0;
      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      // When FSM says CENTER, we already set chosen_armor_position = center above
      if (fsm != AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
        calcYawAndPitch(target_position, rpy_, yaw, pitch);
      }
      break;
    }
  }

  // ── Manual angle offset compensation ────────────────────────────────────
  auto angle_offset = manual_compensator_->angleHardCorrect(
    chosen_armor_position.head(2).norm(), chosen_armor_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180.0;
  double yaw_offset   = angle_offset[1] * M_PI / 180.0;
  double cmd_pitch    = pitch + pitch_offset;
  double cmd_yaw      = angles::normalize_angle(yaw + yaw_offset);

  // ── Fire advice ──────────────────────────────────────────────────────────
  if (state == TRACKING_CENTER || fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
    // When spinning fast, always allow fire toward center
    gimbal_cmd.fire_advice = true;
  } else {
    double center_yaw = std::atan2(target_position.y(), target_position.x());
    double armor_yaw = target_yaw + idx * (2 * M_PI / target.armors_num);
    double diff_yaw = angles::shortest_angular_distance(center_yaw, armor_yaw);
    gimbal_cmd.fire_advice = isOnTarget(target, rpy_[2], rpy_[1], cmd_yaw, cmd_pitch, distance, diff_yaw, fsm);
  }

  gimbal_cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
  gimbal_cmd.pitch      = cmd_pitch * 180.0 / M_PI;
  gimbal_cmd.yaw_diff   = (cmd_yaw   - rpy_[2]) * 180.0 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180.0 / M_PI;

  // ── TinyMPC trajectory smoothing ────────────────────────────────────────
  if (enable_mpc_ && mpc_controller_) {
    // Sync MPC internal state with actual physical gimbal position on reset
    if (reset_mpc_) {
      prev_yaw_cmd_   = rpy_[2];
      prev_pitch_cmd_ = rpy_[1];
      prev_yaw_vel_   = 0.0;
      prev_pitch_vel_ = 0.0;
      reset_mpc_      = false;
    }

    try {
      std::vector<double> ref_yaw_p, ref_yaw_v, ref_pitch_p, ref_pitch_v;
      
      // Angle Unwrapping: Use shortest_angular_distance to ensure cmd_yaw doesn't jump 
      // by 2PI near the boundary, preventing MPC from "spinning the long way around".
      double unwrapped_yaw   = prev_yaw_cmd_   + angles::shortest_angular_distance(prev_yaw_cmd_,   cmd_yaw);
      double unwrapped_pitch = prev_pitch_cmd_ + angles::shortest_angular_distance(prev_pitch_cmd_, cmd_pitch);

      // ── Perfect wust_vision semantic restore ─────────────────────────────────
      // Instead of integrating prev_yaw_cmd_, we create a reference chunk spanning 
      // [-half_horizon, +half_horizon] centered exactly at the predicted target_yaw.
      // ── Calculate dynamic dt for differentiated pitch velocity ──────────
      double real_dt = 0.004; // Fallback to 250Hz (0.004s)
      if (prev_solve_time_.nanoseconds() > 0) {
        double dt_sec = (current_time - prev_solve_time_).seconds();
        // Guard against massive spikes on first frames or large pauses
        real_dt = std::clamp(dt_sec, 0.001, 0.050); 
      }
      prev_solve_time_ = current_time;

      // yaw feedforward: use target angular velocity (directly from tracker)
      double yaw_vel_ff = (fsm == AutoAimFsm::AIM_SINGLE_ARMOR) ? target.v_yaw : 0.0;

      // pitch feedforward: finite difference of cmd_pitch over real inter-call interval
      // Units: (rad - rad) / s = rad/s
      double pitch_vel_ff = angles::shortest_angular_distance(prev_pitch_cmd_, unwrapped_pitch) / real_dt;

      const int    half_horizon = mpc_controller_->params().horizon / 2;
      const double dt_mpc      = mpc_controller_->params().dt;
      double start_yaw   = unwrapped_yaw   - yaw_vel_ff   * (half_horizon * dt_mpc);
      double start_pitch = unwrapped_pitch - pitch_vel_ff  * (half_horizon * dt_mpc);
      
      mpc_controller_->buildConstVelRef(start_yaw,   yaw_vel_ff,   ref_yaw_p,   ref_yaw_v);
      mpc_controller_->buildConstVelRef(start_pitch, pitch_vel_ff, ref_pitch_p, ref_pitch_v);

      // We explicitly bypass prev_yaw_cmd_ and start the solver at the exact start of the reference trajectory!
      // This is exactly what x0 << traj_eigen(0,0) did in very_aimer.cpp.
      auto mpc_res = mpc_controller_->solve(
        ref_yaw_p, ref_yaw_v, ref_pitch_p, ref_pitch_v,
        ref_yaw_p[0], ref_yaw_v[0], 
        ref_pitch_p[0], ref_pitch_v[0],
        /*result_step=*/half_horizon); // Extract the center point (t_hit)

      if (mpc_res.valid) {
        // Output from the MPC centered time directly without integrating!
        const double sy = mpc_res.yaw.p;
        const double sp = mpc_res.pitch.p;
        
        // Store for next iteration (warm-start)
        prev_yaw_vel_   = mpc_res.yaw.v;
        prev_pitch_vel_ = pitch_vel_ff; 
        prev_yaw_cmd_   = sy;
        prev_pitch_cmd_ = sp;

        // Final output: Normalize back to [-PI, PI] for the lower computer
        double norm_sy = angles::normalize_angle(sy);
        double norm_sp = angles::normalize_angle(sp);
        
        gimbal_cmd.yaw        = norm_sy * 180.0 / M_PI;
        gimbal_cmd.pitch      = norm_sp * 180.0 / M_PI;
        gimbal_cmd.yaw_diff   = angles::shortest_angular_distance(rpy_[2], norm_sy) * 180.0 / M_PI;
        gimbal_cmd.pitch_diff = angles::shortest_angular_distance(rpy_[1], norm_sp) * 180.0 / M_PI;

        // MPC feedforward outputs (converted to deg/s and deg/s^2)
        // Both velocity and acceleration come directly from MPC state/input output.
        // No LPF applied — raw MPC output is sent to the lower computer.
        gimbal_cmd.yaw_vel   = mpc_res.yaw.v   * 180.0 / M_PI;
        gimbal_cmd.pitch_vel = mpc_res.pitch.v  * 180.0 / M_PI;
        gimbal_cmd.yaw_acc   = mpc_res.yaw.a   * 180.0 / M_PI;
        gimbal_cmd.pitch_acc = mpc_res.pitch.a * 180.0 / M_PI;
      }
    } catch (const std::exception &e) {
      FYT_WARN("armor_solver", "MPC solve failed: {}", e.what());
      prev_yaw_cmd_   = cmd_yaw;
      prev_pitch_cmd_ = cmd_pitch;
    }
  } else {
    prev_yaw_cmd_   = cmd_yaw;
    prev_pitch_cmd_ = cmd_pitch;
    prev_yaw_vel_   = 0.0;
    prev_pitch_vel_ = 0.0;
  }
  // ────────────────────────────────────────────────────────────────────────

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "Fire! FSM={}", autoAimFsmToString(fsm));
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const rm_interfaces::msg::Target &target,
                        const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance,
                        const double diff_yaw,
                        const AutoAimFsm fsm) const noexcept {
  bool is_big = (target.id == "1" || target.id == "base" || (target.armors_num == 4 && (target.id == "guard" || target.id == "outpost")));
  if (target.id == "3" || target.id == "4" || target.id == "5" || target.id == "2") {
    // Standard armors are typically small
    is_big = false; 
  }
  
  double width = is_big ? 0.23 : 0.135; 
  double height = is_big ? 0.13 : 0.125; 

  double yaw_factor = 1.0;
  if (fsm != AutoAimFsm::AIM_SINGLE_ARMOR) {
    if (std::abs(diff_yaw) <= 30.0 / 180.0 * M_PI) {
      yaw_factor = std::cos(diff_yaw);
    }
  } else {
    yaw_factor = std::cos(diff_yaw);
  }

  double shooting_range_yaw   = std::abs(atan2(width / 2 * yaw_factor, distance));
  double shooting_range_pitch = std::abs(atan2(height / 2, distance));
  shooting_range_yaw   = std::max(shooting_range_yaw,   0.8 * M_PI / 180.0);
  shooting_range_pitch = std::max(shooting_range_pitch, 0.8 * M_PI / 180.0);
  return (std::abs(cur_yaw   - target_yaw)   < shooting_range_yaw &&
          std::abs(cur_pitch - target_pitch) < shooting_range_pitch);
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r         = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r         = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d              &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  double alpha = std::atan2(target_center.y(), target_center.x());
  double beta  = target_yaw;

  Eigen::Matrix2d R_odom2center, R_odom2armor;
  R_odom2center << std::cos(alpha),  std::sin(alpha),
                  -std::sin(alpha),  std::cos(alpha);
  R_odom2armor  << std::cos(beta),   std::sin(beta),
                  -std::sin(beta),   std::cos(beta);
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;
  double decision_angle = -std::asin(R_center2armor(0, 1));
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) theta = 0;

  double temp_angle = decision_angle + M_PI / armors_num - theta;
  if (temp_angle < 0) temp_angle += 2 * M_PI;
  return static_cast<int>(temp_angle / (2 * M_PI / armors_num));
}

// ── selectArmorFsm ──────────────────────────────────────────────────────────
// Ported from wust_vision VeryAimer::Impl::selectArmor()
// delta_angle[i] = normalize( armor_yaw[i] - center_yaw )
//   ≈ angle from car's facing direction to armor i, in car-body frame
int Solver::selectArmorFsm(const std::vector<Eigen::Vector3d> &armor_positions,
                           const Eigen::Vector3d              &target_center,
                           const double target_yaw,
                           const double target_v_yaw,
                           const size_t armors_num,
                           AutoAimFsm   fsm) const noexcept {
  if (armors_num == 0) return 0;
  const int N = static_cast<int>(armors_num);

  // center_yaw: direction from origin to car center (odom frame)
  const double center_yaw = std::atan2(target_center.y(), target_center.x());

  // delta_angle[i] = angular offset of armor i relative to the car's front
  std::vector<double> delta_angles(N);
  for (int i = 0; i < N; ++i) {
    // Armor yaw = target_yaw + i * (2π/N)
    const double armor_yaw = target_yaw + i * (2.0 * M_PI / N);
    delta_angles[i] = angles::normalize_angle(armor_yaw - center_yaw);
  }

  // Helper: pick the index with minimum |delta|
  const auto pick_min_delta = [&](const std::vector<int> &idxs) -> int {
    int best = idxs.empty() ? 0 : idxs[0];
    double best_val = std::numeric_limits<double>::infinity();
    for (int i : idxs) {
      if (std::abs(delta_angles[i]) < best_val) {
        best_val = std::abs(delta_angles[i]);
        best = i;
      }
    }
    return best;
  };

  // ── AIM_SINGLE_ARMOR: lock onto the one armor facing us most directly ──
  if (fsm == AutoAimFsm::AIM_SINGLE_ARMOR) {
    const double in_first = coming_angle_deg_ * M_PI / 180.0;
    std::vector<int> candidates;
    for (int i = 0; i < N; ++i)
      if (std::abs(delta_angles[i]) <= in_first) candidates.push_back(i);

    if (candidates.size() > 1) {
      int a = candidates[0], b = candidates[1];
      // Maintain lock_id to avoid flickering
      if (locked_armor_id_ != a && locked_armor_id_ != b) {
        locked_armor_id_ = (std::abs(delta_angles[a]) < std::abs(delta_angles[b])) ? a : b;
      }
      int pick = (locked_armor_id_ >= 0 && locked_armor_id_ < N)
        ? locked_armor_id_
        : pick_min_delta(candidates);
      return pick;
    } else if (!candidates.empty()) {
      locked_armor_id_ = -1;
      return candidates[0];
    }
    // Fallback: min delta
    std::vector<int> all(N); std::iota(all.begin(), all.end(), 0);
    return pick_min_delta(all);
  }

  // ── AIM_WHOLE_CAR_ARMOR: coming/leaving angle hysteresis ──────────────
  if (fsm == AutoAimFsm::AIM_WHOLE_CAR_ARMOR) {
    const double coming  = coming_angle_deg_   * M_PI / 180.0;
    const double leaving = leaving_angle_deg_  * M_PI / 180.0;

    int best_idx = -1;
    for (int i = 0; i < N; ++i) {
      if (std::abs(delta_angles[i]) > coming) continue;
      // Prefer the armor that is about to face us (hysteresis)
      if (target_v_yaw > 0  && delta_angles[i] < leaving)  best_idx = i;
      if (target_v_yaw < 0  && delta_angles[i] > -leaving)  best_idx = i;
    }
    if (best_idx < 0) {
      std::vector<int> all(N); std::iota(all.begin(), all.end(), 0);
      best_idx = pick_min_delta(all);
    }
    return best_idx;
  }

  // ── AIM_WHOLE_CAR_PAIR: target alternating pair ────────────────────────
  if (fsm == AutoAimFsm::AIM_WHOLE_CAR_PAIR && N == 4) {
    // Pick from the pair at indices (1,3) or (0,2) based on height offset
    // In wust_vision: target.h() > 0 → pair {1,3}, else {0,2}
    // Here we use v_yaw sign as proxy:
    std::vector<int> pair = (target_v_yaw > 0) ? std::vector<int>{1, 3}
                                                : std::vector<int>{0, 2};
    // Validate indices
    std::vector<int> valid_pair;
    for (int i : pair) if (i < N) valid_pair.push_back(i);
    if (!valid_pair.empty()) return pick_min_delta(valid_pair);
  }

  // ── Fallback: minimum delta (also covers CENTER – caller overrides anyway)
  std::vector<int> all(N); std::iota(all.begin(), all.end(), 0);
  return pick_min_delta(all);
}

void Solver::calcYawAndPitch(const Eigen::Vector3d   &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  yaw   = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());
  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  for (auto &p : trajectory) {
    double x = p.first, y = p.second;
    p.first  = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace fyt::auto_aim
