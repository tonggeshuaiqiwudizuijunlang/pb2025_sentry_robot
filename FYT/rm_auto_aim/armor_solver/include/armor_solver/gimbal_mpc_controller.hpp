// GimbalMpcController — MPC-based gimbal trajectory smoother
// Adapted from wust_vision VeryAimer (TinyMPC + QuinticSegment)
// Integrated into FYT armor_solver by Antigravity
//
// State:  x = [position, velocity]  (2x1 per axis)
// Input:  u = [acceleration]        (1x1 per axis)
// Two independent SISO MPC problems: yaw and pitch.

#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

// tinympc headers (compiled as part of armor_solver target)
#include "tinympc/tiny_api.hpp"

namespace fyt::auto_aim {

// ──────────────────────────────────────────────
// QuinticSegment: C2-continuous 5th-order polynomial
// between two (pos, vel, acc) boundary conditions.
// ──────────────────────────────────────────────
struct QuinticBoundary {
  double p = 0.0;  // position
  double v = 0.0;  // velocity
  double a = 0.0;  // acceleration
};

/**
 * Evaluates a quintic Hermite spline on [0, T].
 * Coefficients are solved analytically for C2 continuity.
 */
class QuinticSegment {
public:
  QuinticSegment() = default;

  QuinticSegment(const QuinticBoundary & start, const QuinticBoundary & end, double T) {
    init(start, end, T);
  }

  void init(const QuinticBoundary & s, const QuinticBoundary & e, double T) {
    T_ = T;
    if (T <= 1e-9) {
      for (auto & c : c_) c = 0.0;
      c_[0] = s.p;
      return;
    }
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T;
    // c0..c2 from start BC
    c_[0] = s.p;
    c_[1] = s.v;
    c_[2] = s.a / 2.0;
    // c3..c5 from end BC
    const double d0 = e.p - c_[0] - c_[1] * T - c_[2] * T2;
    const double d1 = e.v - c_[1] - 2.0 * c_[2] * T;
    const double d2 = e.a - 2.0 * c_[2];
    // Solve 3x3 system (Vandermonde-style)
    c_[3] = (10.0 * d0 / T2 - 4.0 * d1 / T + 0.5 * d2) / T;
    c_[4] = (-15.0 * d0 / T3 + 7.0 * d1 / T2 - d2 / T) / T;
    c_[5] = (6.0 * d0 / T4 - 3.0 * d1 / T3 + 0.5 * d2 / T2) / T;
  }

  double pos(double t) const {
    const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    return c_[0] + c_[1] * t + c_[2] * t2 + c_[3] * t3 + c_[4] * t4 + c_[5] * t5;
  }

  double vel(double t) const {
    const double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
    return c_[1] + 2.0 * c_[2] * t + 3.0 * c_[3] * t2 + 4.0 * c_[4] * t3 + 5.0 * c_[5] * t4;
  }

  double acc(double t) const {
    const double t2 = t * t, t3 = t2 * t;
    return 2.0 * c_[2] + 6.0 * c_[3] * t + 12.0 * c_[4] * t2 + 20.0 * c_[5] * t3;
  }

  double duration() const { return T_; }

private:
  double T_ = 0.0;
  std::array<double, 6> c_{};
};

// ──────────────────────────────────────────────
// GimbalMpcController
// ──────────────────────────────────────────────

struct MpcParams {
  // Horizon & time step
  int    horizon    = 20;    // prediction horizon (steps)
  double dt         = 0.005; // time step [s] (matches ~200Hz control loop)

  // State-space: double-integrator  x = [p, v], u = [a]
  //   x(k+1) = A*x(k) + B*u(k)
  //   A = [[1, dt],[0, 1]],  B = [[dt^2/2],[dt]]

  // Cost weights
  double Q_pos = 100.0;  // position tracking weight
  double Q_vel = 1.0;    // velocity tracking weight
  double R_acc = 0.01;   // control effort weight

  // ADMM penalty
  double rho = 1.0;

  // Input bounds [rad/s^2]
  double max_acc_yaw   = 60.0;  // max yaw  acceleration
  double max_acc_pitch = 40.0;  // max pitch acceleration
};

/**
 * GimbalMpcController wraps two independent 2-state TinyMPC problems
 * (yaw and pitch) and provides a smoothed gimbal trajectory.
 *
 * Usage:
 *   1. Construct with MpcParams.
 *   2. Each control cycle: call solve(target_yaw_traj, target_pitch_traj)
 *      to get smoothed position/velocity/acceleration at the current step.
 */
class GimbalMpcController {
public:
  struct AxisResult {
    double p = 0.0;  // smoothed position  [rad]
    double v = 0.0;  // smoothed velocity  [rad/s]
    double a = 0.0;  // smoothed acceleration [rad/s^2]
  };

  struct SolveResult {
    AxisResult yaw;
    AxisResult pitch;
    bool valid = false;
  };

  explicit GimbalMpcController(const MpcParams & params = MpcParams{})
  : params_(params), yaw_solver_(nullptr), pitch_solver_(nullptr) {
    init();
  }

  ~GimbalMpcController() {
    destroySolver(yaw_solver_);
    destroySolver(pitch_solver_);
  }

  /**
   * Solve MPC for one step given reference trajectories.
   *
   * @param ref_yaw_pos   Vector of reference yaw   positions [rad], length = horizon
   * @param ref_yaw_vel   Vector of reference yaw   velocities [rad/s]
   * @param ref_pitch_pos Vector of reference pitch  positions [rad]
   * @param ref_pitch_vel Vector of reference pitch  velocities [rad/s]
   * @param x0_yaw_p   Current yaw   position  [rad]
   * @param x0_yaw_v   Current yaw   velocity  [rad/s]
   * @param x0_pitch_p Current pitch position  [rad]
   * @param x0_pitch_v Current pitch velocity  [rad/s]
   * @param result_step  Which horizon step to return (0 = immediate next)
   */
  SolveResult solve(
    const std::vector<double> & ref_yaw_pos,
    const std::vector<double> & ref_yaw_vel,
    const std::vector<double> & ref_pitch_pos,
    const std::vector<double> & ref_pitch_vel,
    double x0_yaw_p, double x0_yaw_v,
    double x0_pitch_p, double x0_pitch_v,
    int result_step = 0) {
    SolveResult res;
    if (!yaw_solver_ || !pitch_solver_) return res;

    const int N = params_.horizon;
    const int step = std::min(result_step, N - 1);

    // ── Yaw ──
    {
      Eigen::VectorXd x0(2);
      x0 << x0_yaw_p, x0_yaw_v;
      tiny_set_x0(yaw_solver_, x0);

      Eigen::MatrixXd Xref(2, N);
      for (int k = 0; k < N; ++k) {
        Xref(0, k) = (k < (int)ref_yaw_pos.size()) ? ref_yaw_pos[k] : ref_yaw_pos.back();
        Xref(1, k) = (k < (int)ref_yaw_vel.size()) ? ref_yaw_vel[k] : 0.0;
      }
      tiny_set_x_ref(yaw_solver_, Xref);
      tiny_solve(yaw_solver_);

      res.yaw.p = yaw_solver_->work->x(0, step);
      res.yaw.v = yaw_solver_->work->x(1, step);
      res.yaw.a = (step < N - 1) ? yaw_solver_->work->u(0, step) : 0.0;
    }

    // ── Pitch ──
    {
      Eigen::VectorXd x0(2);
      x0 << x0_pitch_p, x0_pitch_v;
      tiny_set_x0(pitch_solver_, x0);

      Eigen::MatrixXd Xref(2, N);
      for (int k = 0; k < N; ++k) {
        Xref(0, k) = (k < (int)ref_pitch_pos.size()) ? ref_pitch_pos[k] : ref_pitch_pos.back();
        Xref(1, k) = (k < (int)ref_pitch_vel.size()) ? ref_pitch_vel[k] : 0.0;
      }
      tiny_set_x_ref(pitch_solver_, Xref);
      tiny_solve(pitch_solver_);

      res.pitch.p = pitch_solver_->work->x(0, step);
      res.pitch.v = pitch_solver_->work->x(1, step);
      res.pitch.a = (step < N - 1) ? pitch_solver_->work->u(0, step) : 0.0;
    }

    res.valid = true;
    return res;
  }

  /**
   * Build a reference trajectory by forward-simulating the target
   * with constant velocity (simple prediction over the horizon).
   *
   * @param p0  current position [rad]
   * @param v0  current velocity [rad/s]
   * @param pos_out  output positions
   * @param vel_out  output velocities
   */
  void buildConstVelRef(
    double p0, double v0,
    std::vector<double> & pos_out,
    std::vector<double> & vel_out) const {
    const int N = params_.horizon;
    const double dt = params_.dt;
    pos_out.resize(N);
    vel_out.resize(N);
    for (int k = 0; k < N; ++k) {
      pos_out[k] = p0 + v0 * (k * dt);
      vel_out[k] = v0;
    }
  }

  const MpcParams & params() const { return params_; }

private:
  void init() {
    const int N = params_.horizon;
    const double dt = params_.dt;

    // Double-integrator state space
    Eigen::MatrixXd A(2, 2), B(2, 1), f(2, 1);
    A << 1.0, dt,
         0.0, 1.0;
    B << 0.5 * dt * dt,
         dt;
    f.setZero();

    // Cost matrices (diagonal)
    Eigen::MatrixXd Q(2, 2), R(1, 1);
    Q << params_.Q_pos, 0.0,
         0.0,           params_.Q_vel;
    R << params_.R_acc;

    setupSolver(&yaw_solver_,   A, B, f, Q, R, N, params_.max_acc_yaw);
    setupSolver(&pitch_solver_, A, B, f, Q, R, N, params_.max_acc_pitch);
  }

  void setupSolver(
    TinySolver ** solverp,
    const Eigen::MatrixXd & A,
    const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & f,
    const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & R,
    int N,
    double max_acc) {
    int status = tiny_setup(
      solverp, A, B, f, Q, R,
      params_.rho,
      /*nx=*/2, /*nu=*/1, N,
      /*verbose=*/0);
    if (status != 0) {
      *solverp = nullptr;
      return;
    }

    // Acceleration bounds
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, N - 1, -max_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, N - 1,  max_acc);
    // No explicit state bounds (use large values)
    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, N, -1e6);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, N,  1e6);

    tiny_set_bound_constraints(*solverp, x_min, x_max, u_min, u_max);

    // Update settings: enable input bounds, tighten tolerances
    tiny_update_settings(
      (*solverp)->settings,
      /*abs_pri_tol=*/1e-3,
      /*abs_dua_tol=*/1e-3,
      /*max_iter=*/50,
      /*check_termination=*/5,
      /*en_state_bound=*/0,
      /*en_input_bound=*/1,
      /*en_state_soc=*/0,
      /*en_input_soc=*/0,
      /*en_state_linear=*/0,
      /*en_input_linear=*/0);
  }

  static void destroySolver(TinySolver * s) {
    if (!s) return;
    delete s->solution;
    delete s->settings;
    delete s->cache;
    delete s->work;
    delete s;
  }

  MpcParams params_;
  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;
};

}  // namespace fyt::auto_aim
