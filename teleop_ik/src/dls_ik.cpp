#include "teleop_ik/dls_ik.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace teleop_ik
{
namespace
{
// Gradient that pushes each joint toward the middle of its range, scaled so
// it grows sharply near the limits (Liegeois-style cost).
Vec7 jointLimitGradient(const Vec7 & q, const JointLimits & l)
{
  Vec7 g;
  for (int i = 0; i < kJoints; ++i) {
    const double mid = 0.5 * (l.lower(i) + l.upper(i));
    const double range = l.upper(i) - l.lower(i);
    g(i) = -(q(i) - mid) / (range * range);
  }
  return g;
}
}  // namespace

Vec7 dlsStep(
  const Eigen::Isometry3d & target, const Vec7 & q, const JointLimits & limits,
  const IkOptions & opt, IkResult * diag)
{
  const Eigen::Isometry3d current = forwardKinematics(q);
  const Jacobian j_full = jacobian(q);
  const int rows = opt.use_orientation ? 6 : 3;

  Eigen::VectorXd err(rows);
  err.head<3>() = target.translation() - current.translation();
  if (opt.use_orientation) {
    err.tail<3>() = rotationError(target.linear(), current.linear());
  }
  const Eigen::MatrixXd j = j_full.topRows(rows);

  // Damping from the smallest singular value: zero far from singularities,
  // up to max_damping as the arm approaches one.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeFullV);
  const double s_min = svd.singularValues()(rows - 1);
  double lambda2 = 0.0;
  if (s_min < opt.singular_threshold) {
    const double r = s_min / opt.singular_threshold;
    lambda2 = (1.0 - r * r) * opt.max_damping * opt.max_damping;
  }

  const Eigen::MatrixXd jjt = j * j.transpose() + lambda2 * Eigen::MatrixXd::Identity(rows, rows);
  const Eigen::MatrixXd j_pinv = j.transpose() * jjt.ldlt().solve(Eigen::MatrixXd::Identity(rows, rows));
  Vec7 dq = j_pinv * err;

  // Secondary task in the null space: does not move the hand (to first order).
  // The projector comes from the SVD, not from the damped inverse: with
  // damping, I - J_pinv J is no longer an exact projector, and the
  // joint-limit push leaks into the task and stalls convergence.
  const Eigen::MatrixXd v_range = svd.matrixV().leftCols(rows);
  const Eigen::Matrix<double, kJoints, kJoints> nullspace =
    Eigen::Matrix<double, kJoints, kJoints>::Identity() - v_range * v_range.transpose();
  dq += nullspace * (opt.nullspace_gain * jointLimitGradient(q, limits));

  // Cap the step, preserving direction.
  const double biggest = dq.cwiseAbs().maxCoeff();
  if (biggest > opt.max_step) {
    dq *= opt.max_step / biggest;
  }

  Vec7 q_next = q + dq;
  for (int i = 0; i < kJoints; ++i) {
    q_next(i) = std::clamp(q_next(i), limits.lower(i) + opt.limit_margin, limits.upper(i) - opt.limit_margin);
  }

  if (diag) {
    diag->min_singular_value = s_min;
    diag->damping = std::sqrt(lambda2);
  }
  return q_next;
}

IkResult solve(
  const Eigen::Isometry3d & target, const Vec7 & q_seed, const JointLimits & limits, const IkOptions & opt)
{
  IkResult r;
  r.q = q_seed;
  for (r.iterations = 0; r.iterations <= opt.max_iterations; ++r.iterations) {
    const Eigen::Isometry3d current = forwardKinematics(r.q);
    r.position_error = (target.translation() - current.translation()).norm();
    r.orientation_error = rotationError(target.linear(), current.linear()).norm();
    const bool pos_ok = r.position_error < opt.position_tolerance;
    const bool rot_ok = !opt.use_orientation || r.orientation_error < opt.orientation_tolerance;
    if (pos_ok && rot_ok) {
      r.converged = true;
      break;
    }
    if (r.iterations == opt.max_iterations) {
      break;
    }
    r.q = dlsStep(target, r.q, limits, opt, &r);
  }
  return r;
}

IkResult solveWithRestarts(
  const Eigen::Isometry3d & target, const Vec7 & q_seed, const JointLimits & limits,
  const IkOptions & opt, int restarts, unsigned int seed)
{
  IkResult best = solve(target, q_seed, limits, opt);
  std::mt19937 rng(seed);
  for (int k = 0; k < restarts && !best.converged; ++k) {
    Vec7 q;
    for (int i = 0; i < kJoints; ++i) {
      std::uniform_real_distribution<double> u(limits.lower(i) + opt.limit_margin, limits.upper(i) - opt.limit_margin);
      q(i) = u(rng);
    }
    IkResult r = solve(target, q, limits, opt);
    r.iterations += best.iterations;
    if (r.converged || r.position_error < best.position_error) {
      best = r;
    } else {
      best.iterations = r.iterations;
    }
  }
  return best;
}

Vec7 velocityLimitedStep(
  const Eigen::Isometry3d & target, const Vec7 & q, double dt, const JointLimits & limits,
  const IkOptions & opt, IkResult * diag)
{
  Vec7 q_next = dlsStep(target, q, limits, opt, diag);
  Vec7 dq = q_next - q;
  double scale = 1.0;
  for (int i = 0; i < kJoints; ++i) {
    const double allowed = limits.max_velocity(i) * dt;
    if (std::abs(dq(i)) > allowed) {
      scale = std::min(scale, allowed / std::abs(dq(i)));
    }
  }
  return q + scale * dq;  // uniform scaling keeps the hand moving in a straight line
}

}  // namespace teleop_ik
