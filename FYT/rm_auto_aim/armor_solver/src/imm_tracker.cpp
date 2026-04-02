// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// IMM-EKF Tracker Implementation
//
// References:
// [1] Blom & Bar-Shalom (1988). IEEE TAC 33(8).
// [2] Bar-Shalom et al. (2001). Estimation. Wiley, Ch. 11.

#include "armor_solver/imm_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

// ============================================================
// Constructor
// ============================================================
IMMTracker::IMMTracker(
  const Eigen::Matrix<double, NUM_MODELS, NUM_MODELS>& transition_prob,
  double sage_husa_b0)
: pi_(transition_prob)
, sage_husa_b0_(sage_husa_b0) {
  // Initialize Sage-Husa estimators with the forgetting factor
  for (int i = 0; i < NUM_MODELS; ++i) {
    sage_husa_[i] = RobotSageHusa(sage_husa_b0_);
  }

  // Default initial model probabilities (uniform)
  double uniform = 1.0 / NUM_MODELS;
  for (int i = 0; i < NUM_MODELS; ++i) {
    models_[i].mu = uniform;
    models_[i].likelihood = 1.0;
  }

  x_fused_ = VectorX::Zero();
  P_fused_ = MatrixXX::Identity();
}

// ============================================================
// Init
// ============================================================
void IMMTracker::init(const VectorX& x0, const MatrixXX& P0) noexcept {
  for (int i = 0; i < NUM_MODELS; ++i) {
    models_[i].x = x0;
    models_[i].P = P0;
    models_[i].x_pred = x0;
    models_[i].P_pred = P0;
    models_[i].mu = 1.0 / NUM_MODELS;
  }
  x_fused_ = x0;
  P_fused_ = P0;
  initialized_ = true;

  for (auto& sh : sage_husa_) {
    sh.reset();
  }
}

// ============================================================
// IMM Predict Step
// Steps 1 (mixing) + 2 (per-model predict)
// ============================================================
void IMMTracker::predict(double dt) noexcept {
  if (!initialized_) return;

  // ---- Step 1a: Predicted mixing probabilities c_bar(i|j) = pi_{ij} * mu_i ----
  // c_bar[j][i] = pi[i][j] * mu[i]     (i=from, j=to)
  Eigen::Matrix<double, NUM_MODELS, NUM_MODELS> c_bar;
  for (int j = 0; j < NUM_MODELS; ++j) {
    double norm = 0.0;
    for (int i = 0; i < NUM_MODELS; ++i) {
      c_bar(i, j) = pi_(i, j) * models_[i].mu;
      norm += c_bar(i, j);
    }
    // Normalize column j
    if (norm > 1e-12) {
      for (int i = 0; i < NUM_MODELS; ++i) {
        c_bar(i, j) /= norm;
      }
    }
  }

  // ---- Step 1b: State mixing for each model j ----
  std::array<VectorX, NUM_MODELS> x_prev, x_mix;
  std::array<MatrixXX, NUM_MODELS> P_prev, P_mix;
  for (int i = 0; i < NUM_MODELS; ++i) {
    x_prev[i] = models_[i].x;
    P_prev[i] = models_[i].P;
  }
  mixStates(x_prev, P_prev, c_bar, x_mix, P_mix);

  // ---- Step 2: Per-model prediction ----
  for (int j = 0; j < NUM_MODELS; ++j) {
    predictPerModel(j, dt, x_mix[j], P_mix[j]);
  }

  // Store mixed state as bridge for next cycle's mixing step
  // (models_[i].x will be overwritten by update() with the posterior)
  for (int i = 0; i < NUM_MODELS; ++i) {
    models_[i].x = x_mix[i];
  }

  // ---- CRITICAL FIX: Update x_fused_ to the predicted (a priori) fused state ----
  // Without this, getState() between predict() and update() returns the OLD posterior.
  // On the first predict() call after init(), x_fused_ would still be x0 (fine),
  // but on subsequent calls it would return the previous cycle's posterior,
  // which is only slightly stale. The real problem is that IMMTracker::init()
  // sets x_fused_ = x0 correctly, but if predict() is called before the first
  // update(), x_fused_ must reflect the predicted position, not the pre-predict value.
  x_fused_ = VectorX::Zero();
  P_fused_ = MatrixXX::Zero();
  for (int j = 0; j < NUM_MODELS; ++j) {
    x_fused_ += models_[j].mu * models_[j].x_pred;
  }
  for (int j = 0; j < NUM_MODELS; ++j) {
    VectorX dx = models_[j].x_pred - x_fused_;
    P_fused_ += models_[j].mu * (models_[j].P_pred + dx * dx.transpose());
  }
  P_fused_ = 0.5 * (P_fused_ + P_fused_.transpose());

  // Final safety clamp on fused state
  x_fused_[8] = std::clamp(x_fused_[8], 0.10, 0.45);
  x_fused_[9] = std::clamp(x_fused_[9], -0.5,  0.5);
}

// ============================================================
// IMM Update Step
// Steps 3 (per-model update) + 4 (prob update) + 5 (fusion)
// ============================================================
void IMMTracker::update(const VectorZ& z) noexcept {
  if (!initialized_) return;

  // ---- Step 3: Per-model update + likelihood ----
  for (int j = 0; j < NUM_MODELS; ++j) {
    updatePerModel(j, z);
  }

  // ---- Step 4: Recompute predicted model probs from pi and current mu ----
  //             mu_pred_j = sum_i pi_{ij} * mu_i
  std::array<double, NUM_MODELS> mu_pred;
  for (int j = 0; j < NUM_MODELS; ++j) {
    mu_pred[j] = 0.0;
    for (int i = 0; i < NUM_MODELS; ++i) {
      mu_pred[j] += pi_(i, j) * models_[i].mu;
    }
  }

  // mu_new_j = likelihood_j * mu_pred_j / normalizer
  double norm = 0.0;
  for (int j = 0; j < NUM_MODELS; ++j) {
    norm += models_[j].likelihood * mu_pred[j];
  }
  if (norm < 1e-300) norm = 1e-300;  // avoid divide by zero

  for (int j = 0; j < NUM_MODELS; ++j) {
    models_[j].mu = models_[j].likelihood * mu_pred[j] / norm;
  }

  // ---- Step 5: Fuse estimates ----
  fuseEstimates();
}

// ============================================================
// Step 1b: State Mixing
// For each model j: compute x_mix_j = sum_i c_bar(i|j) * x_i
// and P_mix_j = sum_i c_bar(i|j) * [P_i + (x_i-x_mix_j)*(x_i-x_mix_j)^T]
// ============================================================
void IMMTracker::mixStates(const std::array<VectorX, NUM_MODELS>& x_prev,
                           const std::array<MatrixXX, NUM_MODELS>& P_prev,
                           const Eigen::Matrix<double, NUM_MODELS, NUM_MODELS>& c_bar,
                           std::array<VectorX, NUM_MODELS>& x_mix,
                           std::array<MatrixXX, NUM_MODELS>& P_mix) noexcept {
  for (int j = 0; j < NUM_MODELS; ++j) {
    // Mixed mean
    x_mix[j] = VectorX::Zero();
    for (int i = 0; i < NUM_MODELS; ++i) {
      x_mix[j] += c_bar(i, j) * x_prev[i];
    }

    // Mixed covariance
    P_mix[j] = MatrixXX::Zero();
    for (int i = 0; i < NUM_MODELS; ++i) {
      VectorX dx = x_prev[i] - x_mix[j];
      P_mix[j] += c_bar(i, j) * (P_prev[i] + dx * dx.transpose());
    }
    // Enforce symmetry
    P_mix[j] = 0.5 * (P_mix[j] + P_mix[j].transpose());
  }
}

// ============================================================
// Step 2: Per-model prediction
// Uses corresponding motion model and Q matrix
// ============================================================
void IMMTracker::predictPerModel(int j, double dt,
                                 const VectorX& x_mix,
                                 const MatrixXX& P_mix) noexcept {
  // Build Q for model j
  MatrixXX Q = buildQ(j, dt);

  // Build the process function for model j with the specific dt
  Predict f_j(dt, model_types_[j], singer_alpha_);

  // Use Ceres AutoDiff to compute Jacobian F = df/dx
  // (same approach as the existing EKF)
  using Jet = ceres::Jet<double, X_N>;
  Jet x_e_jet[X_N], x_p_jet[X_N];
  for (int i = 0; i < X_N; ++i) {
    x_e_jet[i].a = x_mix[i];
    x_e_jet[i].v.setZero();
    x_e_jet[i].v[i] = 1.0;
  }
  f_j(x_e_jet, x_p_jet);

  MatrixXX F = MatrixXX::Zero();
  VectorX x_pred = VectorX::Zero();
  for (int i = 0; i < X_N; ++i) {
    x_pred[i] = x_p_jet[i].a;
    F.row(i) = x_p_jet[i].v.transpose();
  }

  // Predicted covariance
  MatrixXX P_pred = F * P_mix * F.transpose() + Q;
  P_pred = 0.5 * (P_pred + P_pred.transpose());

  models_[j].x_pred = x_pred;
  models_[j].P_pred = P_pred;
}

// ============================================================
// Step 3: Per-model EKF update + Gaussian likelihood
// ============================================================
void IMMTracker::updatePerModel(int j, const VectorZ& z) noexcept {
  const VectorX& x_pred = models_[j].x_pred;
  const MatrixXX& P_pred = models_[j].P_pred;

  // Compute H = dh/dx via AutoDiff
  Measure h;
  using Jet = ceres::Jet<double, X_N>;
  Jet x_p_jet[X_N];
  for (int i = 0; i < X_N; ++i) {
    x_p_jet[i].a = x_pred[i];
    x_p_jet[i].v.setZero();
    x_p_jet[i].v[i] = 1.0;
  }
  Jet z_p_jet[Z_N];
  h(x_p_jet, z_p_jet);

  Eigen::Matrix<double, Z_N, X_N> H = Eigen::Matrix<double, Z_N, X_N>::Zero();
  VectorZ z_pred = VectorZ::Zero();
  for (int i = 0; i < Z_N; ++i) {
    z_pred[i] = z_p_jet[i].a;
    H.row(i) = z_p_jet[i].v.transpose();
  }

  // Adaptive R (from Sage-Husa, or use default)
  MatrixZZ R = buildR(z);

  // Innovation
  VectorZ innov = z - z_pred;

  // Innovation covariance — regularize to prevent near-singular inverse
  MatrixZZ S = H * P_pred * H.transpose() + R;
  S += MatrixZZ::Identity() * 1e-6;  // numerical floor

  // Kalman gain
  Eigen::Matrix<double, X_N, Z_N> K =
    P_pred * H.transpose() * S.inverse();

  // Posterior state
  VectorX x_post = x_pred + K * innov;

  // Posterior covariance (Joseph form for numerical stability)
  MatrixXX I_KH = MatrixXX::Identity() - K * H;
  MatrixXX P_post = I_KH * P_pred * I_KH.transpose() + K * R * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());

  // Sage-Husa adaptive noise update
  sage_husa_[j].update(innov, H, P_pred, K, /*Q=*/const_cast<MatrixXX&>(models_[j].P_pred),
                        /*R=*/R);

  // Gaussian likelihood: L = N(z; z_pred, S)
  models_[j].likelihood = gaussianLikelihood(innov, S);
  models_[j].x = x_post;
  models_[j].P = P_post;

  // --- Clamp per-model radius to physically reasonable range ---
  // Prevents any single sub-model's radius from diverging and
  // polluting the weighted fusion (x_fused_[8]).
  models_[j].x[8] = std::clamp(models_[j].x[8], 0.10, 0.45);
  // d_zc should also stay small (height offset < 0.5m)
  models_[j].x[9] = std::clamp(models_[j].x[9], -0.5, 0.5);
}

// ============================================================
// Step 5: Fuse estimates
// x_fused = sum_j mu_j * x_j
// P_fused = sum_j mu_j * [P_j + (x_j - x_fused)*(x_j - x_fused)^T]
// ============================================================
void IMMTracker::fuseEstimates() noexcept {
  x_fused_ = VectorX::Zero();
  for (int j = 0; j < NUM_MODELS; ++j) {
    x_fused_ += models_[j].mu * models_[j].x;
  }

  P_fused_ = MatrixXX::Zero();
  for (int j = 0; j < NUM_MODELS; ++j) {
    VectorX dx = models_[j].x - x_fused_;
    P_fused_ += models_[j].mu * (models_[j].P + dx * dx.transpose());
  }
  P_fused_ = 0.5 * (P_fused_ + P_fused_.transpose());
}

// ============================================================
// Build Q matrix for model j
// Model 0 (CV):   standard Singer process noise (smaller)
// Model 1 (CA):   larger velocity noise to allow fast acceleration
// Model 2 (CTRV): larger yaw_dot noise for spinning targets
// ============================================================
IMMTracker::MatrixXX IMMTracker::buildQ(int j, double dt) const noexcept {
  MatrixXX Q = MatrixXX::Zero();

  // Common: Singer noise blocks for position+velocity (xy, z separately)
  double sigma_a2_xy = s2q_xy_;
  double sigma_a2_z  = s2q_z_;
  double sigma_a2_yaw= s2q_yaw_;
  double sigma_r     = s2q_r_ * std::pow(dt, 4) / 4.0;
  double sigma_dzc   = s2q_r_ * std::pow(dt, 4) / 4.0;

  // CA and CTRV models get scaled-up noise
  if (j == 1) {
    // CA: much larger xy acceleration uncertainty
    sigma_a2_xy *= s2q_ca_factor_;
  } else if (j == 2) {
    // CTRV: larger yaw angular acceleration uncertainty
    sigma_a2_yaw *= s2q_ca_factor_ * 2.0;
  }

  // Singer noise block for X (pos + vel)
  double alpha_xy  = singer_alpha_;
  auto Qxy = SingerNoiseBlock::compute(dt, alpha_xy, sigma_a2_xy);
  Q(0, 0) = Qxy(0, 0);  Q(0, 1) = Qxy(0, 1);
  Q(1, 0) = Qxy(1, 0);  Q(1, 1) = Qxy(1, 1);

  // Singer noise block for Y
  Q(2, 2) = Qxy(0, 0);  Q(2, 3) = Qxy(0, 1);
  Q(3, 2) = Qxy(1, 0);  Q(3, 3) = Qxy(1, 1);

  // Singer noise for Z (usually smaller)
  auto Qz = SingerNoiseBlock::compute(dt, alpha_xy, sigma_a2_z);
  Q(4, 4) = Qz(0, 0);  Q(4, 5) = Qz(0, 1);
  Q(5, 4) = Qz(1, 0);  Q(5, 5) = Qz(1, 1);

  // Yaw random walk
  double q_yaw_yaw   = std::pow(dt, 4) / 4.0 * sigma_a2_yaw;
  double q_yaw_vyaw  = std::pow(dt, 3) / 2.0 * sigma_a2_yaw;
  double q_vyaw_vyaw = std::pow(dt, 2)        * sigma_a2_yaw;
  Q(6, 6) = q_yaw_yaw;   Q(6, 7) = q_yaw_vyaw;
  Q(7, 6) = q_yaw_vyaw;  Q(7, 7) = q_vyaw_vyaw;

  Q(8, 8) = sigma_r;
  Q(9, 9) = sigma_dzc;

  return Q;
}

// ============================================================
// Build R (measurement noise covariance)
// Distance-adaptive: larger R when far away
// ============================================================
IMMTracker::MatrixZZ IMMTracker::buildR(const VectorZ& z) const noexcept {
  MatrixZZ R = MatrixZZ::Zero();
  R(0, 0) = r_xyz_ * std::abs(z[0]) + 1e-4;  // xa
  R(1, 1) = r_xyz_ * std::abs(z[1]) + 1e-4;  // ya
  R(2, 2) = r_xyz_ * std::abs(z[2]) + 1e-4;  // za
  R(3, 3) = r_yaw_;                           // yaw
  return R;
}

// ============================================================
// Gaussian likelihood:  L = (2pi)^(-NZ/2) * |S|^(-0.5) * exp(-0.5*v^T*S^-1*v)
// ============================================================
double IMMTracker::gaussianLikelihood(const VectorZ& innov,
                                      const MatrixZZ& S) const noexcept {
  constexpr double kTwoPi = 2.0 * M_PI;
  double det = S.determinant();
  if (det <= 0.0) det = 1e-300;

  double exponent = -0.5 * innov.transpose() * S.inverse() * innov;
  double coeff = 1.0 / (std::pow(kTwoPi, Z_N / 2.0) * std::sqrt(det));
  double L = coeff * std::exp(exponent);

  // Clamp to avoid numerical underflow
  return std::max(L, 1e-300);
}

// ============================================================
// Public getters
// ============================================================
IMMTracker::VectorX IMMTracker::getState() const noexcept {
  return x_fused_;
}

IMMTracker::MatrixXX IMMTracker::getCovariance() const noexcept {
  return P_fused_;
}

std::array<double, IMMTracker::NUM_MODELS> IMMTracker::getModelProbs() const noexcept {
  std::array<double, NUM_MODELS> probs;
  for (int j = 0; j < NUM_MODELS; ++j) {
    probs[j] = models_[j].mu;
  }
  return probs;
}

int IMMTracker::getDominantModel() const noexcept {
  int best = 0;
  for (int j = 1; j < NUM_MODELS; ++j) {
    if (models_[j].mu > models_[best].mu) best = j;
  }
  return best;
}

bool IMMTracker::isSpinning(double spin_threshold) const noexcept {
  // Model 2 (CTRV) dominant → target is spinning
  return models_[2].mu > spin_threshold;
}

double IMMTracker::getVyaw() const noexcept {
  return x_fused_[7];  // v_yaw from fused state
}

void IMMTracker::resetAdaptiveNoise() noexcept {
  for (auto& sh : sage_husa_) {
    sh.reset();
  }
}

void IMMTracker::setNoiseParams(double s2q_xy, double s2q_z, double s2q_yaw,
                                double s2q_r, double s2q_ca_factor,
                                double r_xyz, double r_yaw) noexcept {
  s2q_xy_       = s2q_xy;
  s2q_z_        = s2q_z;
  s2q_yaw_      = s2q_yaw;
  s2q_r_        = s2q_r;
  s2q_ca_factor_= s2q_ca_factor;
  r_xyz_        = r_xyz;
  r_yaw_        = r_yaw;
}

}  // namespace fyt::auto_aim
