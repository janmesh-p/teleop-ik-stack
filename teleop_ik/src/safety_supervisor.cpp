#include "teleop_ik/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_ik
{

const char * modeName(Mode m)
{
  switch (m) {
    case Mode::kIdle: return "IDLE";
    case Mode::kActive: return "ACTIVE";
    case Mode::kHold: return "HOLD";
    case Mode::kFault: return "FAULT";
    case Mode::kEstop: return "ESTOP";
  }
  return "?";
}

Vec7 frankaMaxAcceleration()
{
  Vec7 a;
  a << 15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0;
  return a;
}

SafetySupervisor::SafetySupervisor(const JointLimits & limits, const SafetyConfig & config)
: limits_(limits), cfg_(config)
{
  if (cfg_.max_acceleration.isZero() || !cfg_.max_acceleration.allFinite()) {
    cfg_.max_acceleration = frankaMaxAcceleration();
  }
}

void SafetySupervisor::enter(Mode m, const std::string & reason, CycleOutput & out)
{
  if (m != mode_) {
    out.transition = true;
  }
  mode_ = m;
  reason_ = reason;
}

void SafetySupervisor::clampToLimits()
{
  for (int i = 0; i < kJoints; ++i) {
    const double lo = limits_.lower(i) + cfg_.limit_margin, hi = limits_.upper(i) - cfg_.limit_margin;
    if (q_cmd_(i) <= lo || q_cmd_(i) >= hi) {
      q_cmd_(i) = std::clamp(q_cmd_(i), lo, hi);
      v_cmd_(i) = 0.0;
    }
  }
}

void SafetySupervisor::rampDown(double dt)
{
  // Each joint decelerates at its limit; all stop without overshoot.
  for (int i = 0; i < kJoints; ++i) {
    const double dv = cfg_.max_acceleration(i) * dt;
    v_cmd_(i) = std::abs(v_cmd_(i)) <= dv ? 0.0 : v_cmd_(i) - std::copysign(dv, v_cmd_(i));
  }
  q_cmd_ += v_cmd_ * dt;
  clampToLimits();
}

void SafetySupervisor::track(const Vec7 & q_target, double dt)
{
  // Follow the proposal within velocity and acceleration limits. Speed is
  // also capped so the joint can always brake in time: the discrete-time
  // stopping condition v^2/(2a) + v dt/2 <= |error| solved for v. The
  // continuous form sqrt(2 a |error|) ignores the step and overshoots.
  for (int i = 0; i < kJoints; ++i) {
    const double err = q_target(i) - q_cmd_(i);
    const double a = cfg_.max_acceleration(i);
    const double h = 0.5 * a * dt;
    const double v_brake = std::sqrt(h * h + 2.0 * a * std::abs(err)) - h;
    const double v_want = std::copysign(std::min(std::abs(err) / dt, v_brake), err);
    const double dv = cfg_.max_acceleration(i) * dt;
    double v = std::clamp(v_want, v_cmd_(i) - dv, v_cmd_(i) + dv);
    v = std::clamp(v, -limits_.max_velocity(i), limits_.max_velocity(i));
    v_cmd_(i) = v;
  }
  q_cmd_ += v_cmd_ * dt;
  clampToLimits();
}

CycleOutput SafetySupervisor::step(const CycleInput & in, double dt)
{
  CycleOutput out;
  const bool robot_ok = in.have_joints && in.joints_age <= cfg_.robot_stale_s;
  if (!initialized_ && robot_ok) {
    q_cmd_ = in.q_measured;
    initialized_ = true;
  }

  bool limit_violation = false;
  if (in.have_joints) {
    for (int i = 0; i < kJoints; ++i) {
      limit_violation |= in.q_measured(i) < limits_.lower(i) || in.q_measured(i) > limits_.upper(i);
    }
  }

  // Following error: where the arm is vs where it was told to be.
  bool follow_fault = false;
  if ((mode_ == Mode::kActive || mode_ == Mode::kHold) && robot_ok) {
    const double err =
      (forwardKinematics(in.q_measured).translation() - forwardKinematics(q_cmd_).translation()).norm();
    if (err > cfg_.follow_error_m) {
      if (follow_error_since_ < 0) {follow_error_since_ = in.now;}
      follow_fault = in.now - follow_error_since_ >= cfg_.follow_error_s;
    } else {
      follow_error_since_ = -1.0;
    }
  } else {
    follow_error_since_ = -1.0;
  }

  // Transitions, highest priority first.
  if (estop_pressed_) {
    if (mode_ != Mode::kEstop) {enter(Mode::kEstop, "e-stop pressed", out);}
  } else if (mode_ == Mode::kEstop || mode_ == Mode::kFault) {
    if (reset_request_) {
      if (robot_ok && !limit_violation) {
        engage_request_ = false;  // reset never resumes motion by itself
        enter(Mode::kIdle, "reset", out);
      }
    }
  } else if (mode_ == Mode::kActive || mode_ == Mode::kHold) {
    if (!robot_ok) {
      enter(Mode::kFault, "robot state lost", out);
    } else if (limit_violation) {
      enter(Mode::kFault, "joint limit exceeded", out);
    } else if (follow_fault) {
      enter(Mode::kFault, "following error", out);
    } else if (!engage_request_) {
      enter(Mode::kIdle, "disengaged", out);
    } else if (in.marker_age > cfg_.operator_lost_s) {
      engage_request_ = false;
      enter(Mode::kIdle, "operator lost", out);
    } else if (in.marker_age > cfg_.marker_hold_s) {
      if (mode_ != Mode::kHold) {enter(Mode::kHold, "marker lost", out);}
    } else if (mode_ != Mode::kActive) {
      enter(Mode::kActive, "marker back", out);
    }
  } else if (mode_ == Mode::kIdle && engage_request_) {
    const bool stopped = v_cmd_.cwiseAbs().maxCoeff() < cfg_.stopped_velocity;
    if (robot_ok && !limit_violation && in.marker_age <= cfg_.marker_hold_s && stopped) {
      q_cmd_ = in.q_measured;  // resync to where the arm actually is
      v_cmd_.setZero();
      enter(Mode::kActive, "engaged", out);
    } else {
      engage_request_ = false;
      enter(Mode::kIdle, "engage refused", out);
      out.transition = true;  // report the refusal even though the mode is unchanged
    }
  }
  reset_request_ = false;

  if (mode_ == Mode::kActive) {
    track(in.q_proposed, dt);
  } else {
    rampDown(dt);
  }

  const bool moving = v_cmd_.cwiseAbs().maxCoeff() >= cfg_.stopped_velocity;
  out.mode = mode_;
  out.reason = reason_;
  out.q_cmd = q_cmd_;
  out.v_cmd = v_cmd_;
  // Idle with the arm at rest: stay silent, so another controller may own
  // the arm. Every other mode keeps commanding, including the hold position.
  out.publish = initialized_ && (mode_ != Mode::kIdle || moving);
  return out;
}

}  // namespace teleop_ik
