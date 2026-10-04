#pragma once

#include "teleop_ik/franka_kinematics.hpp"

namespace teleop_ik
{

struct IkOptions
{
  bool use_orientation = false;     // position-only leaves 4 redundant DoF
  double position_tolerance = 1e-3;     // m
  double orientation_tolerance = 1e-2;  // rad
  int max_iterations = 200;
  // Damping rises smoothly once the smallest singular value of the task
  // Jacobian drops below singular_threshold (Maciejewski-Klein style).
  double max_damping = 0.08;
  double singular_threshold = 0.04;
  double max_step = 0.15;           // rad per iteration, largest joint change
  double nullspace_gain = 0.3;      // joint-limit avoidance strength
  double limit_margin = 0.02;       // rad kept clear of each hard limit
};

struct IkResult
{
  Vec7 q;
  bool converged = false;
  int iterations = 0;
  double position_error = 0.0;     // m
  double orientation_error = 0.0;  // rad
  double min_singular_value = 0.0;
  double damping = 0.0;
};

/// One damped least-squares step, with null-space joint-limit avoidance,
/// a per-step size cap, and hard joint-limit clamping. Returns the new q.
Vec7 dlsStep(
  const Eigen::Isometry3d & target, const Vec7 & q, const JointLimits & limits,
  const IkOptions & opt, IkResult * diag = nullptr);

/// Iterate dlsStep from q_seed until within tolerance or max_iterations.
IkResult solve(
  const Eigen::Isometry3d & target, const Vec7 & q_seed, const JointLimits & limits,
  const IkOptions & opt = IkOptions());

/// solve() from q_seed, then from up to `restarts` random seeds inside the
/// limits if that fails. Returns the first converged result, else the one
/// with the smallest position error. Local IK gets trapped by joint limits
/// and large orientation changes; random restarts are the standard remedy.
IkResult solveWithRestarts(
  const Eigen::Isometry3d & target, const Vec7 & q_seed, const JointLimits & limits,
  const IkOptions & opt, int restarts, unsigned int seed = 0);

/// Streaming teleop step: move from q toward the IK solution for target,
/// never faster than the joint velocity limits allow in dt seconds.
Vec7 velocityLimitedStep(
  const Eigen::Isometry3d & target, const Vec7 & q, double dt, const JointLimits & limits,
  const IkOptions & opt, IkResult * diag = nullptr);

}  // namespace teleop_ik
