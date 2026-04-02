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
//
// IMM (Interacting Multiple Model) Tracker
// Implements the IMM-EKF algorithm for maneuvering target tracking.
//
// References:
// [1] Blom, H.A.P. & Bar-Shalom, Y. (1988). "The interacting multiple model
//     algorithm for systems with Markovian switching coefficients."
//     IEEE Transactions on Automatic Control, 33(8), 780-783.
// [2] Bar-Shalom et al. (2001). "Estimation with Applications to Tracking
//     and Navigation." Wiley. Chapter 11.
// [3] Zhang, W. et al. (2024). "Improved IMM-SOEKF for maneuvering extended
//     target tracking." IEEE Sensors Journal.

#ifndef ARMOR_SOLVER_IMM_TRACKER_HPP_
#define ARMOR_SOLVER_IMM_TRACKER_HPP_

// std
#include <array>
#include <functional>
#include <memory>
#include <vector>
// Eigen
#include <Eigen/Dense>
// Ceres
#include <ceres/jet.h>
// project
#include "armor_solver/motion_model.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::auto_aim {

// ============================================================
// IMM (Interacting Multiple Model) EKF
//
// Models used:
//   Model 0 (CV):    Constant Velocity + Constant Rotation
//   Model 1 (CA):    Constant Acceleration approximation (larger Q)
//   Model 2 (CTRV):  High-rotation model (Singer for yaw)
//
// State dimension: X_N = 10
// Measurement dimension: Z_N = 4
// ============================================================
class IMMTracker {
public:
  static constexpr int NUM_MODELS = 3;

  using VectorX = Eigen::Matrix<double, X_N, 1>;
  using MatrixXX = Eigen::Matrix<double, X_N, X_N>;
  using VectorZ = Eigen::Matrix<double, Z_N, 1>;
  using MatrixZZ = Eigen::Matrix<double, Z_N, Z_N>;

  // Model state container
  struct SubModel {
    VectorX x;       // posterior state estimate
    MatrixXX P;      // posterior error covariance
    double mu;       // model probability
    VectorX x_pred;  // a priori state
    MatrixXX P_pred; // a priori covariance
    double likelihood;  // current measurement likelihood
  };

  // Constructor
  // transition_prob: NUM_MODELS x NUM_MODELS Markov transition matrix (row-normalized)
  explicit IMMTracker(
    const Eigen::Matrix<double, NUM_MODELS, NUM_MODELS>& transition_prob,
    double sage_husa_b0 = 0.96);

  // Initialize from first detection
  void init(const VectorX& x0, const MatrixXX& P0) noexcept;

  // IMM prediction step (call before update)
  void predict(double dt) noexcept;

  // IMM update step (call after predict, with measurement)
  // measurement: [xa, ya, za, yaw_armor]
  void update(const VectorZ& z) noexcept;

  // Get fused state estimate
  VectorX getState() const noexcept;

  // Get fused error covariance
  MatrixXX getCovariance() const noexcept;

  // Get model probabilities [mu_0, mu_1, mu_2]
  std::array<double, NUM_MODELS> getModelProbs() const noexcept;

  // Get index of dominant model
  int getDominantModel() const noexcept;

  // Check if currently in high-spin model (CTRV dominant)
  bool isSpinning(double spin_threshold = 0.5) const noexcept;

  // Get estimated angular velocity (from fused state)
  double getVyaw() const noexcept;

  // Reset Sage-Husa adaptive noise counters
  void resetAdaptiveNoise() noexcept;

  // Set process noise parameters (called when parameters change)
  void setNoiseParams(double s2q_xy, double s2q_z, double s2q_yaw,
                      double s2q_r, double s2q_ca_factor,
                      double r_xyz, double r_yaw) noexcept;

private:
  // ---- IMM Steps ----

  // Step 1: State mixing (interaction)
  // Computes mixed initial conditions for each filter
  void mixStates(const std::array<VectorX, NUM_MODELS>& x_prev,
                 const std::array<MatrixXX, NUM_MODELS>& P_prev,
                 const Eigen::Matrix<double, NUM_MODELS, NUM_MODELS>& c_bar,
                 std::array<VectorX, NUM_MODELS>& x_mix,
                 std::array<MatrixXX, NUM_MODELS>& P_mix) noexcept;

  // Step 2: Per-model prediction (using different Q per model)
  void predictPerModel(int model_idx, double dt,
                       const VectorX& x_mix, const MatrixXX& P_mix) noexcept;

  // Step 3: Per-model update + likelihood computation
  void updatePerModel(int model_idx, const VectorZ& z) noexcept;

  // Step 4: Model probability update
  void updateModelProbs(const Eigen::Matrix<double, NUM_MODELS, NUM_MODELS>& c_bar) noexcept;

  // Step 5: Fusion
  void fuseEstimates() noexcept;

  // Build Q matrix for a given model and dt
  MatrixXX buildQ(int model_idx, double dt) const noexcept;

  // Build R matrix (measurement noise)
  MatrixZZ buildR(const VectorZ& z) const noexcept;

  // Compute Gaussian likelihood
  double gaussianLikelihood(const VectorZ& innov, const MatrixZZ& S) const noexcept;

  // ---- Data Members ----
  std::array<SubModel, NUM_MODELS> models_;

  // Markov transition probability matrix (rows: from model i, cols: to model j)
  Eigen::Matrix<double, NUM_MODELS, NUM_MODELS> pi_;

  // Fused output
  VectorX x_fused_;
  MatrixXX P_fused_;

  // Sage-Husa adaptive noise per model
  std::array<RobotSageHusa, NUM_MODELS> sage_husa_;

  // EKF process/measurement functions per model
  // Model 0: CVROT (standard), Model 1: CA (larger Q), Model 2: CTRV (high spin)
  std::array<MotionModel, NUM_MODELS> model_types_ = {
    MotionModel::CONSTANT_VEL_ROT,
    MotionModel::CONSTANT_ACCEL,
    MotionModel::SINGER
  };

  // Noise parameters
  double s2q_xy_{20.0};
  double s2q_z_{20.0};
  double s2q_yaw_{100.0};
  double s2q_r_{800.0};
  double s2q_ca_factor_{5.0};  // CA model gets s2q_xy * ca_factor
  double r_xyz_{0.05};
  double r_yaw_{0.02};
  double singer_alpha_{0.1};   // 1/tau_m, used for SINGER model
  double sage_husa_b0_{0.96};

  // Initialized flag
  bool initialized_{false};
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_IMM_TRACKER_HPP_
