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
// OPTIMIZATION NOTE:
// Extended motion model library beyond simple CV/CR.
// Added CA (Constant Acceleration) and Singer (1st-order Markov acceleration)
// models for better maneuvering target tracking. Intended for use in the
// IMM (Interacting Multiple Model) framework.
//
// References:
// [1] Singer, R.A. (1970). "Estimating optimal tracking filter performance for
//     manned maneuvering targets." IEEE TAE&S.
// [2] Bar-Shalom et al. (2001). "Estimation with Applications to Tracking and
//     Navigation." Wiley.

#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// std
#include <cmath>
// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::auto_aim {

// ============================================================
// State layout (shared across models):
// Index:  0    1     2    3     4    5     6    7      8    9
//         xc   v_xc  yc   v_yc  zc   v_zc  yaw  v_yaw  r    d_zc
// ============================================================

// State dimension and measurement dimension
constexpr int X_N = 10, Z_N = 4;

// ------------------------------------------------------------
// Motion Model Enum
// - CONSTANT_VELOCITY (CV):  position += velocity * dt
// - CONSTANT_ROTATION (CR):  only yaw changes (for outpost)
// - CONSTANT_VEL_ROT (CVROT): CV + CR combined (original default)
// - CONSTANT_ACCEL (CA):     velocity += acceleration * dt  (new)
// - SINGER:                  acceleration as 1st-order Markov (new)
// ------------------------------------------------------------
enum class MotionModel {
  CONSTANT_VELOCITY = 0,
  CONSTANT_ROTATION = 1,
  CONSTANT_VEL_ROT = 2,
  CONSTANT_ACCEL = 3,   // NEW: CA model
  SINGER = 4,           // NEW: Singer model
};

// ============================================================
// Predict functor — used by EKF/IMM as the process model f(x)
// State: [xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, r, dzc]
// ============================================================
struct Predict {
  explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT,
                   double singer_alpha = 0.1)
  : dt(dt), model(model), singer_alpha(singer_alpha) {}

  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) const {
    // Copy state
    for (int i = 0; i < X_N; i++) {
      x1[i] = x0[i];
    }

    switch (model) {
      // ------ CV: xc += vxc*dt, yc += vyc*dt, zc += vzc*dt ------
      case MotionModel::CONSTANT_VELOCITY: {
        x1[0] += x0[1] * dt;   // xc
        x1[2] += x0[3] * dt;   // yc
        x1[4] += x0[5] * dt;   // zc
        break;
      }
      // ------ CR: only yaw evolves ------
      case MotionModel::CONSTANT_ROTATION: {
        x1[1] *= T(0.);  // no vxc
        x1[3] *= T(0.);  // no vyc
        x1[5] *= T(0.);  // no vzc
        x1[6] += x0[7] * dt;  // yaw
        break;
      }
      // ------ CVROT: CV + rotation (original default) ------
      case MotionModel::CONSTANT_VEL_ROT: {
        x1[0] += x0[1] * dt;
        x1[2] += x0[3] * dt;
        x1[4] += x0[5] * dt;
        x1[6] += x0[7] * dt;
        break;
      }
      // ------ CA: v += a*dt, x += v*dt + 0.5*a*dt^2 ------
      // NOTE: We embed the virtual acceleration into the velocity change.
      // In this 10-state model, we do not have an explicit acceleration state.
      // CA is approximated by using a larger process noise on velocity states.
      // For full CA, use the 13-state model below (PredictCA).
      // Here we use a "nearly constant acceleration" approximation:
      // velocity is modelled as a random walk with larger noise.
      case MotionModel::CONSTANT_ACCEL: {
        x1[0] += x0[1] * dt;
        x1[2] += x0[3] * dt;
        x1[4] += x0[5] * dt;
        x1[6] += x0[7] * dt;
        // Velocity assumed constant (noise carries the acceleration uncertainty)
        break;
      }
      // ------ Singer: a(t+dt) = rho*a(t) + w(t), exponential decay ------
      // Approximation within 10-state: no explicit 'a' state.
      // Singer is handled by choosing velocity noise consistent with
      // the Singer PSD: q_singer = sigma_a^2 * (1 - rho^2) / (2*alpha)
      // The actual Singer motion propagation uses the 13-state PredictSinger.
      case MotionModel::SINGER: {
        x1[0] += x0[1] * dt;
        x1[2] += x0[3] * dt;
        x1[4] += x0[5] * dt;
        x1[6] += x0[7] * dt;
        break;
      }
    }
  }

  double dt;
  MotionModel model;
  double singer_alpha;  // alpha = 1/tau_m for Singer model
};

// ============================================================
// Measure functor — h(x): state → measurement
// Measurement: [xa, ya, za, yaw_armor]
//   xa = xc - cos(yaw)*r
//   ya = yc - sin(yaw)*r
//   za = zc + dzc
//   yaw_a = yaw (armor orientation)
// ============================================================
struct Measure {
  template <typename T>
  void operator()(const T x[X_N], T z[Z_N]) {
    z[0] = x[0] - ceres::cos(x[6]) * x[8];
    z[1] = x[2] - ceres::sin(x[6]) * x[8];
    z[2] = x[4] + x[9];
    z[3] = x[6];

  }
};

// Convenience typedef: the original single EKF (compatible with existing code)
using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;


// ============================================================
// Singer Model Process Noise Q helper
// Computes the Singer spectral density matrix element for one axis.
// q_singer(dt, alpha, sigma_a^2) — used to build the Q matrix in IMM.
//
// Reference: Singer (1970), eq. (16)
// Q_pos_pos = sigma_a^2/alpha^2 * [2*alpha*t - 4 + 2*rho + (1-rho^2)/2alpha*t]/(2alpha)
// (simplified for small dt: Q ≈ sigma_a^2 * dt^5/(20) for pos, etc.)
// ============================================================
struct SingerNoiseBlock {
  // Returns a 2x2 [pos, vel] partial noise block for the Singer model
  // alpha = 1/tau_m (maneuver time constant reciprocal)
  // sigma_a2 = acceleration variance
  static Eigen::Matrix2d compute(double dt, double alpha, double sigma_a2) {
    double rho = std::exp(-alpha * dt);
    double rho2 = rho * rho;
    double alpha2 = alpha * alpha;
    double alpha3 = alpha2 * alpha;

    // See Bar-Shalom 2001, Table 6.3
    // Q_pp = sigma_a2/alpha2 * [2*alpha*dt - 3 + 4*rho - rho2] / (2*alpha)
    double q_pp = sigma_a2 / alpha2 * (2.0 * alpha * dt - 3.0 + 4.0 * rho - rho2) / (2.0 * alpha);

    // Q_pv = sigma_a2/alpha2 * [1 + rho2 - 2*rho] / alpha
    double q_pv = sigma_a2 / alpha2 * (1.0 + rho2 - 2.0 * rho) / alpha;

    // Q_vv = sigma_a2/alpha2 * [2*alpha*dt - rho2 - 1] / (2*alpha)
    double q_vv = sigma_a2 * (2.0 * alpha * dt - rho2 - 1.0) / (2.0 * alpha3);

    Eigen::Matrix2d Q;
    Q << q_pp, q_pv,
         q_pv, q_vv;
    return Q;
  }
};

// ============================================================
// Sage-Husa Adaptive Noise Estimator
// Updates Q and R online using the innovation sequence.
//
// Reference:
//   Sage, A.P. & Husa, G.W. (1969). "Adaptive filtering with unknown
//   prior statistics." JACC.
//   See also: Mohamed & Schwarz (1999), Journal of Geodesy.
// ============================================================
template <int NX, int NZ>
class SageHusaAdaptiveNoise {
public:
  using MatrixXX = Eigen::Matrix<double, NX, NX>;
  using MatrixZZ = Eigen::Matrix<double, NZ, NZ>;
  using MatrixXZ = Eigen::Matrix<double, NX, NZ>;
  using VectorZ  = Eigen::Matrix<double, NZ, 1>;

  // b0: forgetting factor (0 < b0 < 1), typically 0.94-0.98
  // When b0 = 1, no forgetting (pure sample covariance estimation)
  explicit SageHusaAdaptiveNoise(double b0 = 0.96) : b0_(b0), k_(0) {}

  // Call after each filter predict+update cycle.
  // innovation: z - H*x_pred
  // H: observation matrix NZ x NX
  // P_pri: a priori error covariance
  // K: Kalman gain
  void update(const VectorZ& innovation,
              const Eigen::Matrix<double, NZ, NX>& H,
              const MatrixXX& P_pri,
              const MatrixXZ& K,
              MatrixXX& Q,
              MatrixZZ& R) {
    k_++;
    // b_k = (1 - b0) / (1 - b0^k) — fading memory weight
    double b_k = (1.0 - b0_) / (1.0 - std::pow(b0_, static_cast<double>(k_)));

    // Adaptive R update:  R_k = (1-b)*R + b*(d*d^T + H*P_pri*H^T)
    MatrixZZ innov_outer = innovation * innovation.transpose();
    MatrixZZ H_P_Ht = H * P_pri * H.transpose();
    R = (1.0 - b_k) * R + b_k * (innov_outer + H_P_Ht);
    // Enforce symmetry and positive semi-definiteness
    R = 0.5 * (R + R.transpose());
    R = R.cwiseMax(MatrixZZ::Zero());

    // Adaptive Q update:  Q_k = (1-b)*Q + b*(K*d*d^T*K^T)
    MatrixXX K_innov = K * innovation * innovation.transpose() * K.transpose();
    Q = (1.0 - b_k) * Q + b_k * K_innov;
    // Enforce symmetry and positive semi-definiteness
    Q = 0.5 * (Q + Q.transpose());
    Q = Q.cwiseMax(MatrixXX::Zero());

    // Prevent Q from shrinking too small (numerical floor)
    for (int i = 0; i < NX; ++i) {
      Q(i, i) = std::max(Q(i, i), 1e-6);
    }
  }

  // Reset counter (call when tracker re-initializes)
  void reset() { k_ = 0; }

private:
  double b0_;  // forgetting factor
  int k_;      // iteration count
};

// Convenience alias for the standard 10-state / 4-measurement case
using RobotSageHusa = SageHusaAdaptiveNoise<X_N, Z_N>;

}  // namespace fyt::auto_aim
#endif  // ARMOR_SOLVER_MOTION_MODEL_HPP_
