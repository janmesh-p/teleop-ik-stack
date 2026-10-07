#include <gtest/gtest.h>

#include "teleop_ik/dls_ik.hpp"
#include "teleop_ik/safety_supervisor.hpp"

using namespace teleop_ik;

namespace
{
constexpr double kDt = 0.01;

struct Rig
{
  SafetySupervisor sup{frankaLimits(), SafetyConfig()};
  CycleInput in;
  double t = 0.0;

  Rig()
  {
    in.have_joints = true;
    in.joints_age = 0.0;
    in.marker_age = 0.0;
    in.q_measured = frankaHome();
    in.q_proposed = frankaHome();
  }
  CycleOutput tick()
  {
    t += kDt;
    in.now = t;
    CycleOutput o = sup.step(in, kDt);
    // Ideal robot: follows the command exactly.
    in.q_measured = o.q_cmd;
    return o;
  }
  CycleOutput run(double seconds)
  {
    CycleOutput o;
    for (int k = 0; k < static_cast<int>(seconds / kDt + 0.5); ++k) {o = tick();}
    return o;
  }
  void engage()
  {
    sup.requestEngage(true);
    ASSERT_EQ(tick().mode, Mode::kActive);
  }
};
}  // namespace

TEST(Safety, IdleIsSilentAndEngageNeedsFreshInputs)
{
  Rig r;
  EXPECT_FALSE(r.tick().publish);
  r.in.marker_age = 1.0;  // marker not visible
  r.sup.requestEngage(true);
  CycleOutput o = r.tick();
  EXPECT_EQ(o.mode, Mode::kIdle);
  EXPECT_EQ(o.reason, "engage refused");
  r.in.marker_age = 0.0;
  r.engage();
}

TEST(Safety, ActiveFollowsProposalWithinAccelerationLimits)
{
  Rig r;
  r.engage();
  r.in.q_proposed(0) += 0.5;  // a big jump
  Vec7 v_prev = Vec7::Zero();
  for (int k = 0; k < 100; ++k) {
    CycleOutput o = r.tick();
    for (int i = 0; i < kJoints; ++i) {
      ASSERT_LE(std::abs(o.v_cmd(i) - v_prev(i)), frankaMaxAcceleration()(i) * kDt + 1e-9);
      ASSERT_LE(std::abs(o.v_cmd(i)), frankaLimits().max_velocity(i) + 1e-9);
    }
    v_prev = o.v_cmd;
  }
  EXPECT_NEAR(r.sup.commanded()(0), r.in.q_proposed(0), 1e-3);
}

TEST(Safety, TrackingNeverOvershootsTarget)
{
  Rig r;
  r.engage();
  const double start = r.sup.commanded()(0);
  r.in.q_proposed(0) = start + 0.5;
  double peak = start;
  for (int k = 0; k < 200; ++k) {peak = std::max(peak, r.tick().q_cmd(0));}
  // Discrete braking can overshoot by at most a dt^2 / 8 (0.19 mrad here).
  EXPECT_LE(peak, start + 0.5 + frankaMaxAcceleration()(0) * kDt * kDt / 8 + 1e-9);
  EXPECT_NEAR(r.sup.commanded()(0), start + 0.5, 1e-4);
}

TEST(Safety, EstopLatchesUntilReleasedAndReset)
{
  Rig r;
  r.engage();
  r.in.q_proposed(0) += 0.5;
  r.run(0.2);  // moving
  r.sup.setEstop(true);
  CycleOutput o = r.tick();
  EXPECT_EQ(o.mode, Mode::kEstop);
  o = r.run(0.5);
  EXPECT_LT(o.v_cmd.cwiseAbs().maxCoeff(), 1e-9) << "must come to rest";
  r.sup.requestReset();  // still pressed: reset ignored
  EXPECT_EQ(r.tick().mode, Mode::kEstop);
  r.sup.setEstop(false);
  EXPECT_EQ(r.tick().mode, Mode::kEstop) << "release alone must not clear it";
  r.sup.requestReset();
  EXPECT_EQ(r.tick().mode, Mode::kIdle);
  EXPECT_EQ(r.run(0.5).mode, Mode::kIdle) << "reset must not resume motion";
}

TEST(Safety, RampDownStopsWithinAccelerationBound)
{
  Rig r;
  r.engage();
  r.in.q_proposed(0) += 1.0;
  const CycleOutput o0 = r.run(0.15);
  const double v0 = std::abs(o0.v_cmd(0));
  ASSERT_GT(v0, 0.5);
  r.sup.setEstop(true);
  int cycles = 0;
  while (r.tick().v_cmd.cwiseAbs().maxCoeff() > 0 && cycles < 1000) {++cycles;}
  const double bound = v0 / frankaMaxAcceleration()(0) / kDt + 2;
  EXPECT_LE(cycles, bound);
}

TEST(Safety, RobotStateLossFaultsAndLatches)
{
  Rig r;
  r.engage();
  r.in.joints_age = 0.15;
  CycleOutput o = r.tick();
  EXPECT_EQ(o.mode, Mode::kFault);
  EXPECT_EQ(o.reason, "robot state lost");
  r.sup.requestReset();
  EXPECT_EQ(r.tick().mode, Mode::kFault) << "cannot reset while the robot is still unseen";
  r.in.joints_age = 0.0;
  r.sup.requestReset();
  EXPECT_EQ(r.tick().mode, Mode::kIdle);
}

TEST(Safety, MarkerLossHoldsThenResumesThenDisengages)
{
  Rig r;
  r.engage();
  r.in.marker_age = 0.3;
  EXPECT_EQ(r.tick().mode, Mode::kHold);
  r.in.marker_age = 0.0;
  EXPECT_EQ(r.tick().mode, Mode::kActive) << "short dropout: resume automatically";
  r.in.marker_age = 2.5;
  CycleOutput o = r.tick();
  EXPECT_EQ(o.mode, Mode::kIdle);
  EXPECT_EQ(o.reason, "operator lost");
  r.in.marker_age = 0.0;
  EXPECT_EQ(r.run(0.5).mode, Mode::kIdle) << "operator lost needs a fresh engage";
}

TEST(Safety, FollowingErrorFaultsAfterPersistence)
{
  Rig r;
  r.engage();
  CycleOutput o;
  for (int k = 0; k < 40; ++k) {
    r.t += kDt;
    r.in.now = r.t;
    Vec7 stuck = frankaHome();
    stuck(1) += 0.3;  // arm blocked somewhere else: hand ~10 cm away
    r.in.q_measured = stuck;
    o = r.sup.step(r.in, kDt);
    if (r.t < 0.29) {ASSERT_EQ(o.mode, Mode::kActive) << "must not trip on a transient";}
  }
  EXPECT_EQ(o.mode, Mode::kFault);
  EXPECT_EQ(o.reason, "following error");
}

TEST(Safety, MeasuredJointBeyondLimitFaults)
{
  Rig r;
  r.engage();
  r.t += kDt;
  r.in.now = r.t;
  r.in.q_measured(3) = frankaLimits().upper(3) + 0.01;
  EXPECT_EQ(r.sup.step(r.in, kDt).reason, "joint limit exceeded");
}

TEST(Safety, CommandsNeverLeaveLimits)
{
  Rig r;
  r.engage();
  r.in.q_proposed = frankaLimits().upper.array() + 1.0;
  for (int k = 0; k < 300; ++k) {
    const CycleOutput o = r.tick();
    for (int i = 0; i < kJoints; ++i) {
      ASSERT_LE(o.q_cmd(i), frankaLimits().upper(i));
    }
  }
}

TEST(Safety, StreamingTeleopThroughSupervisorIsNotThrottled)
{
  // Regression: feeding the supervisor one-cycle IK steps made its braking
  // logic cap joint speed near sqrt(2 a v dt) and the hand lagged by up to
  // 47 mm on this sweep. With the solved IK target it stays within 15 mm.
  IkOptions opt;
  opt.use_orientation = true;
  opt.max_iterations = 10;
  Eigen::Isometry3d start = forwardKinematics(frankaHome());
  start.translation() << 0.50, 0.0, 0.41;
  IkOptions o0 = opt;
  o0.max_iterations = 500;
  const Vec7 q0 = solve(start, frankaHome(), frankaLimits(), o0).q;

  SafetySupervisor sup(frankaLimits(), SafetyConfig());
  CycleInput in;
  in.have_joints = true;
  in.joints_age = 0.0;
  in.marker_age = 0.0;
  in.q_measured = q0;
  in.q_proposed = q0;
  sup.requestEngage(true);
  ASSERT_EQ(sup.step(in, kDt).mode, Mode::kActive);

  double worst = 0.0;
  for (int k = 0; k < 1200; ++k) {
    const double t = k * kDt;
    Eigen::Isometry3d target = start;
    target.translation() << 0.50 + 0.13 * std::sin(2 * M_PI * t / 4.0), 0.10 * std::sin(2 * M_PI * t / 3.0),
      0.41 + 0.04 * std::sin(2 * M_PI * t / 2.5);
    in.now = t;
    in.q_proposed = solve(target, sup.commanded(), frankaLimits(), opt).q;
    const CycleOutput o = sup.step(in, kDt);
    in.q_measured = o.q_cmd;
    if (k > 200) {
      worst = std::max(worst, (forwardKinematics(o.q_cmd).translation() - target.translation()).norm());
    }
  }
  std::printf("streaming sweep through supervisor: worst position error %.1f mm\n", worst * 1000);
  EXPECT_LT(worst, 0.015);
}
