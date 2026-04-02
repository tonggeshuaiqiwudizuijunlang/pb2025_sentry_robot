// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#include "armor_solver/armor_tracker.hpp"
// std
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, tracked_id(std::string(""))
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(X_N))  // Must be X_N=10, not 9
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, last_yaw_(0) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept {
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  FYT_INFO("armor_solver", "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  if (tracked_armor.type == "large" &&
      (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept {
  // ---- Predict ----
  // Use IMM or EKF depending on the use_imm flag
  Eigen::VectorXd prediction;
  if (use_imm && imm) {
    // IMM: predict step is called externally before update()
    // (dt is set in armor_solver_node via imm->predict(dt))
    prediction = imm->getState();
  } else {
    prediction = ekf->predict();
  }

  bool matched = false;
  target_state = prediction;

  if (!armors_msg->armors.empty()) {
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    for (const auto &armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - prediction(6));
          tracked_armor = armor;
          if (tracked_armor.type == "large" &&
              (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
            tracked_armors_num = ArmorsNum::BALANCE_2;
          } else if (tracked_id == "outpost") {
            tracked_armors_num = ArmorsNum::OUTPOST_3;
          } else {
            tracked_armors_num = ArmorsNum::NORMAL_4;
          }
        }
      }
    }

    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      matched = true;
      auto p = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

      if (use_imm && imm) {
        // IMM update
        imm->update(measurement);
        target_state = imm->getState();
      } else {
        // Original EKF update
        target_state = ekf->update(measurement);
      }
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      handleArmorJump(same_id_armor);
    } else {
      FYT_WARN("armor_solver", "No matched armor found!");
    }
  }

  // Constrain radius within physical bounds
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    if (use_imm && imm) {
      // IMM: propagate constraint into each sub-model state
      // (simplified: just clamp fused state; sub-models converge over time)
    } else {
      ekf->setState(target_state);
    }
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    if (!use_imm) {
      ekf->setState(target_state);
    }
  }

  // Tracking state machine (unchanged)
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        FYT_DEBUG("armor_solver", "Tracker state: TRACKING {} [{}]", tracked_id,
                  use_imm ? "IMM" : "EKF");
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
      FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
      FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
        FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
      FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept {
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(X_N);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
}

void Tracker::initIMM(const Armor &a) noexcept {
  if (!imm) return;
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  Eigen::Matrix<double, X_N, 1> x0;
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  d_za = 0; d_zc = 0; another_r = r;
  x0 << xc, 0, yc, 0, za, 0, yaw, 0, r, 0.0;

  Eigen::Matrix<double, X_N, X_N> P0 = Eigen::Matrix<double, X_N, X_N>::Identity();
  P0.diagonal() << 0.5, 5.0, 0.5, 5.0, 0.5, 5.0, 0.2, 10.0, 0.05, 0.05;

  imm->init(x0, P0);
  target_state = x0;
}

void Tracker::handleArmorJump(const Armor &current_armor) noexcept {
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4) {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    FYT_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // zc
    target_state(5) = 0;                   // vzc
    target_state(9) = d_zc;                // d_zc
    FYT_WARN("armor_solver", "State wrong!");
  }

  ekf->setState(target_state);
  // Also sync IMM if active — without this, post-jump IMM prediction diverges
  if (use_imm && imm) {
    Eigen::Matrix<double, X_N, X_N> P_reset = Eigen::Matrix<double, X_N, X_N>::Identity();
    P_reset.diagonal() << 0.5, 5.0, 0.5, 5.0, 0.5, 5.0, 0.2, 10.0, 0.05, 0.05;
    imm->init(target_state, P_reset);
  }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept {
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept {
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace fyt::auto_aim
