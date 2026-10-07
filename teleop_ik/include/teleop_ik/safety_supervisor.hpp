#pragma once

#include <string>

#include "teleop_ik/franka_kinematics.hpp"

namespace teleop_ik
{

enum class Mode { kIdle, kActive, kHold, kFault, kEstop };
const char * modeName(Mode m);

struct SafetyConfig
{
  double marker_hold_s = 0.2;       // marker missing this long: hold position
  double operator_lost_s = 2.0;     // missing this long: disengage
  double robot_stale_s = 0.1;       // joint states older than this: robot link lost
  double follow_error_m = 0.08;     // measured hand vs commanded hand
  double follow_error_s = 0.3;      // ... sustained this long: fault
  double limit_margin = 0.02;       // commanded joints stay this far inside limits, rad
  double stopped_velocity = 1e-3;   // rad/s, below this a ramp-down is complete
  Vec7 max_acceleration = Vec7::Zero();  // rad/s^2; zero means use the Franka datasheet values
};

/// Franka acceleration limits from the datasheet.
Vec7 frankaMaxAcceleration();

struct CycleInput
{
  double now = 0.0;                 // s, monotonic
  bool have_joints = false;
  double joints_age = 1e9;          // s since the last joint state
  double marker_age = 1e9;          // s since the last marker pose
  Vec7 q_measured = Vec7::Zero();
  Vec7 q_proposed = Vec7::Zero();   // IK proposal, used only in kActive
};

struct CycleOutput
{
  Mode mode = Mode::kIdle;
  std::string reason;
  Vec7 q_cmd = Vec7::Zero();
  Vec7 v_cmd = Vec7::Zero();
  bool publish = false;             // false: send nothing this cycle
  bool transition = false;          // mode changed this cycle
};

/// Deterministic authority over arm motion. The IK proposes joint targets;
/// the supervisor decides what is actually commanded. Faults and e-stop
/// latch: a reset only returns to idle, and motion needs a fresh engage.
class SafetySupervisor
{
public:
  SafetySupervisor(const JointLimits & limits, const SafetyConfig & config);

  void requestEngage(bool on) { engage_request_ = on; }
  void setEstop(bool pressed) { estop_pressed_ = pressed; }
  void requestReset() { reset_request_ = true; }

  CycleOutput step(const CycleInput & in, double dt);

  Mode mode() const { return mode_; }
  const Vec7 & commanded() const { return q_cmd_; }

private:
  void enter(Mode m, const std::string & reason, CycleOutput & out);
  void rampDown(double dt);
  void track(const Vec7 & q_target, double dt);
  void clampToLimits();

  JointLimits limits_;
  SafetyConfig cfg_;
  Mode mode_ = Mode::kIdle;
  std::string reason_ = "start";
  Vec7 q_cmd_ = Vec7::Zero();
  Vec7 v_cmd_ = Vec7::Zero();
  bool initialized_ = false;
  bool engage_request_ = false;
  bool estop_pressed_ = false;
  bool reset_request_ = false;
  double follow_error_since_ = -1.0;
};

}  // namespace teleop_ik
