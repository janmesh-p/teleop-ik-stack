#include <gtest/gtest.h>

#include <chrono>
#include <random>
#include <vector>
#include <algorithm>

#include "teleop_ik/dls_ik.hpp"

using namespace teleop_ik;

namespace
{
Vec7 randomConfig(std::mt19937 & rng, const JointLimits & l, double margin = 0.15)
{
  Vec7 q;
  for (int i = 0; i < kJoints; ++i) {
    std::uniform_real_distribution<double> u(l.lower(i) + margin, l.upper(i) - margin);
    q(i) = u(rng);
  }
  return q;
}
}  // namespace

TEST(Kinematics, ZeroConfigurationMatchesPublishedFlangePosition)
{
  // Widely published: at q = 0 the Panda flange sits at (0.088, 0, 0.926) m.
  const Eigen::Vector3d p = forwardKinematics(Vec7::Zero()).translation();
  EXPECT_NEAR(p.x(), 0.088, 1e-9);
  EXPECT_NEAR(p.y(), 0.0, 1e-9);
  EXPECT_NEAR(p.z(), 0.926, 1e-9);
}

TEST(Kinematics, RotationIsOrthonormal)
{
  std::mt19937 rng(1);
  const auto l = frankaLimits();
  for (int k = 0; k < 50; ++k) {
    const Eigen::Matrix3d r = forwardKinematics(randomConfig(rng, l)).linear();
    EXPECT_NEAR((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
  }
}

TEST(Kinematics, JacobianMatchesFiniteDifferences)
{
  std::mt19937 rng(2);
  const auto l = frankaLimits();
  const double h = 1e-6;
  for (int k = 0; k < 30; ++k) {
    const Vec7 q = randomConfig(rng, l);
    const Jacobian j = jacobian(q);
    const Eigen::Isometry3d t0 = forwardKinematics(q);
    for (int i = 0; i < kJoints; ++i) {
      Vec7 qh = q;
      qh(i) += h;
      const Eigen::Isometry3d t1 = forwardKinematics(qh);
      const Eigen::Vector3d dp = (t1.translation() - t0.translation()) / h;
      const Eigen::Vector3d dw = rotationError(t1.linear(), t0.linear()) / h;
      EXPECT_NEAR((dp - j.block<3, 1>(0, i)).norm(), 0.0, 1e-5);
      EXPECT_NEAR((dw - j.block<3, 1>(3, i)).norm(), 0.0, 1e-5);
    }
  }
}

TEST(Ik, PositionOnlyReachesRandomReachableTargets)
{
  std::mt19937 rng(3);
  const auto l = frankaLimits();
  IkOptions opt;
  int ok = 0;
  const int n = 500;
  for (int k = 0; k < n; ++k) {
    const Eigen::Isometry3d target = forwardKinematics(randomConfig(rng, l));
    const IkResult r = solve(target, frankaHome(), l, opt);
    ok += r.converged;
    for (int i = 0; i < kJoints; ++i) {
      ASSERT_GE(r.q(i), l.lower(i));
      ASSERT_LE(r.q(i), l.upper(i));
    }
  }
  std::printf("position-only: %d / %d converged\n", ok, n);
  EXPECT_GE(ok, static_cast<int>(0.95 * n));
}

TEST(Ik, FullPoseReachesRandomReachableTargets)
{
  std::mt19937 rng(4);
  const auto l = frankaLimits();
  IkOptions opt;
  opt.use_orientation = true;
  int single = 0, restarted = 0;
  std::vector<double> ms;
  const int n = 500;
  for (int k = 0; k < n; ++k) {
    const Eigen::Isometry3d target = forwardKinematics(randomConfig(rng, l));
    single += solve(target, frankaHome(), l, opt).converged;
    const auto t0 = std::chrono::steady_clock::now();
    const IkResult r = solveWithRestarts(target, frankaHome(), l, opt, 10, k);
    ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    restarted += r.converged;
  }
  std::sort(ms.begin(), ms.end());
  std::printf("full pose: single seed %d / %d, with up to 10 restarts %d / %d | solve p50 %.2f p95 %.2f p99 %.2f ms\n",
    single, n, restarted, n, ms[n / 2], ms[n * 95 / 100], ms[n * 99 / 100]);
  EXPECT_GE(restarted, static_cast<int>(0.98 * n));
}

TEST(Ik, StreamingTracksSmallTargetMotionFromCurrentPose)
{
  // Teleop case: the target moves a little each cycle, IK starts from the
  // current joints. One damped step per 10 ms cycle must keep up.
  const auto l = frankaLimits();
  IkOptions opt;
  opt.use_orientation = true;
  Vec7 q = frankaHome();
  const Eigen::Isometry3d start = forwardKinematics(q);
  double worst = 0.0;
  std::vector<double> us;
  for (int k = 0; k < 400; ++k) {  // 4 s circle, radius 10 cm
    const double a = 2 * M_PI * k / 400.0;
    Eigen::Isometry3d target = start;
    target.translation() += Eigen::Vector3d(0.0, 0.1 * std::sin(a), 0.1 * (1 - std::cos(a)));
    const auto t0 = std::chrono::steady_clock::now();
    q = velocityLimitedStep(target, q, 0.01, l, opt);
    us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    if (k > 20) {
      worst = std::max(worst, (forwardKinematics(q).translation() - target.translation()).norm());
    }
  }
  std::sort(us.begin(), us.end());
  std::printf("streaming circle: worst tracking error %.1f um | step p50 %.1f p99 %.1f us\n",
    worst * 1e6, us[us.size() / 2], us[us.size() * 99 / 100]);
  EXPECT_LT(worst, 0.003);
}

TEST(Ik, UnreachableTargetStaysBoundedAndFinite)
{
  const auto l = frankaLimits();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() << 2.0, 0.0, 0.5;  // far beyond the 0.855 m reach
  const IkResult r = solve(target, frankaHome(), l);
  EXPECT_FALSE(r.converged);
  EXPECT_TRUE(r.q.allFinite());
  EXPECT_GT(r.damping, 0.0) << "stretched arm is near-singular, damping must engage";
}

TEST(Ik, NullSpaceMovesJointsTowardMidRangeWithoutMovingHand)
{
  const auto l = frankaLimits();
  Vec7 q = frankaHome();
  q(0) = l.upper(0) - 0.1;  // joint 1 near its limit
  const Eigen::Isometry3d target = forwardKinematics(q);  // hand already on target
  IkOptions opt;
  opt.nullspace_gain = 2.0;
  Vec7 qi = q;
  for (int k = 0; k < 200; ++k) {
    qi = dlsStep(target, qi, l, opt);
  }
  EXPECT_LT(qi(0), q(0) - 0.05) << "joint 1 should move away from its limit";
  EXPECT_LT((forwardKinematics(qi).translation() - target.translation()).norm(), 2e-3);
}

TEST(Ik, VelocityLimitedStepRespectsLimits)
{
  std::mt19937 rng(5);
  const auto l = frankaLimits();
  IkOptions opt;
  const double dt = 0.01;
  for (int k = 0; k < 100; ++k) {
    const Vec7 q = randomConfig(rng, l);
    const Eigen::Isometry3d far_target = forwardKinematics(randomConfig(rng, l));
    const Vec7 qn = velocityLimitedStep(far_target, q, dt, l, opt);
    for (int i = 0; i < kJoints; ++i) {
      ASSERT_LE(std::abs(qn(i) - q(i)), l.max_velocity(i) * dt + 1e-12);
    }
  }
}
