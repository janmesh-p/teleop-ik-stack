// Marker-driven teleoperation of the Franka in Isaac Sim.
//
// Clutch model: publish true on /teleop/engage and the marker's current
// position becomes the zero point and the hand's current pose the anchor.
// Marker displacement (camera frame) maps to hand displacement (robot base
// frame); hand orientation is held at the anchor. Each control cycle runs one
// velocity-limited damped least-squares IK step from the last commanded
// joints and publishes the result.
//
// A deterministic safety supervisor owns the final command: the IK only
// proposes. E-stop and faults latch; /teleop/reset returns to idle and motion
// needs a fresh engage.
//
// The control loop runs on its own thread with fixed deadlines, not on the
// ROS executor, so callback load cannot stretch the cycle.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "teleop_ik/dls_ik.hpp"
#include "teleop_ik/safety_supervisor.hpp"

using namespace std::chrono;
using teleop_ik::Vec7;

namespace
{
double wallNow()
{
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

// "+x", "-z" etc. -> signed unit row selecting a camera axis.
Eigen::RowVector3d axisRow(const std::string & spec)
{
  if (spec.size() != 2 || (spec[0] != '+' && spec[0] != '-') || spec[1] < 'x' || spec[1] > 'z') {
    throw std::invalid_argument("axis spec must look like +x or -z, got " + spec);
  }
  Eigen::RowVector3d r = Eigen::RowVector3d::Zero();
  r(spec[1] - 'x') = spec[0] == '+' ? 1.0 : -1.0;
  return r;
}

std::string pct(std::vector<double> v, double scale, const char * unit)
{
  if (v.empty()) {return "n/a";}
  std::sort(v.begin(), v.end());
  char b[96];
  std::snprintf(b, sizeof(b), "p50 %.2f p99 %.2f max %.2f %s",
    v[v.size() / 2] * scale, v[v.size() * 99 / 100] * scale, v.back() * scale, unit);
  return b;
}
}  // namespace

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_ik")
  {
    rate_hz_ = declare_parameter("rate_hz", 100.0);
    scale_ = declare_parameter("scale", 1.0);
    stale_s_ = declare_parameter("marker_stale_ms", 200.0) / 1000.0;
    ws_min_ = Eigen::Vector3d(
      declare_parameter("workspace_min_x", 0.20), declare_parameter("workspace_min_y", -0.50),
      declare_parameter("workspace_min_z", 0.08));
    ws_max_ = Eigen::Vector3d(
      declare_parameter("workspace_max_x", 0.75), declare_parameter("workspace_max_y", 0.50),
      declare_parameter("workspace_max_z", 0.85));
    // Operator faces the camera; robot base frame is x forward, y left, z up.
    // Default: marker toward camera -> forward, operator's right -> robot's
    // right, up -> up. Camera optical frame is x right, y down, z forward.
    map_.row(0) = axisRow(declare_parameter("robot_x_from", std::string("-z")));
    map_.row(1) = axisRow(declare_parameter("robot_y_from", std::string("+x")));
    map_.row(2) = axisRow(declare_parameter("robot_z_from", std::string("-y")));
    const std::string log_path = declare_parameter("log_path", std::string("/ws/results/teleop_log.csv"));

    limits_ = teleop_ik::frankaLimits();
    opt_.use_orientation = true;
    // Warm-started from the last command, the solve converges in 1-3
    // iterations; 10 bounds the worst case.
    opt_.max_iterations = declare_parameter("ik_max_iterations", 10);
    teleop_ik::SafetyConfig sc;
    sc.marker_hold_s = stale_s_;
    sc.robot_stale_s = declare_parameter("robot_stale_ms", 100.0) / 1000.0;
    sc.follow_error_m = declare_parameter("follow_error_m", 0.08);
    sc.follow_error_s = declare_parameter("follow_error_ms", 300.0) / 1000.0;
    sup_ = std::make_unique<teleop_ik::SafetySupervisor>(limits_, sc);

    for (int i = 0; i < teleop_ik::kJoints; ++i) {
      joint_names_.push_back("panda_joint" + std::to_string(i + 1));
    }

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states_fast", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m) { onJoints(*m); });
    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/operator/marker_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { onMarker(*m); });
    engage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/teleop/engage", 10, [this](std_msgs::msg::Bool::SharedPtr m) { req_engage_ = m->data ? 1 : 0; });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/teleop/estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) { req_estop_ = m->data ? 1 : 0; });
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/teleop/reset", 10, [this](std_msgs::msg::Bool::SharedPtr m) { if (m->data) {req_reset_ = true;} });
    state_pub_ = create_publisher<std_msgs::msg::String>("/teleop/state", 10);

    log_.open(log_path);
    log_ << "t_cycle,t_marker_capture,t_marker_rx,engaged,stale,clamped,"
            "target_x,target_y,target_z,hand_x,hand_y,hand_z,ik_us,t_publish,min_sv,damping,mode,joints_age_ms\n";

    report_timer_ = create_wall_timer(seconds(2), [this] { report(); });
    loop_ = std::thread([this] { controlLoop(); });
    RCLCPP_INFO(get_logger(), "teleop at %.0f Hz, scale %.2f, log %s. Topics: /teleop/engage /teleop/estop /teleop/reset (Bool), state on /teleop/state.",
      rate_hz_, scale_, log_path.c_str());
  }

  ~TeleopNode() override
  {
    running_ = false;
    if (loop_.joinable()) {loop_.join();}
  }

private:
  void onJoints(const sensor_msgs::msg::JointState & m)
  {
    Vec7 q;
    for (int i = 0; i < teleop_ik::kJoints; ++i) {
      auto it = std::find(m.name.begin(), m.name.end(), joint_names_[i]);
      if (it == m.name.end()) {return;}
      q(i) = m.position[std::distance(m.name.begin(), it)];
    }
    std::lock_guard<std::mutex> lk(mu_);
    q_measured_ = q;
    joints_rx_t_ = wallNow();
    have_joints_ = true;
  }

  void onMarker(const geometry_msgs::msg::PoseStamped & m)
  {
    std::lock_guard<std::mutex> lk(mu_);
    marker_ = Eigen::Vector3d(m.pose.position.x, m.pose.position.y, m.pose.position.z);
    marker_capture_t_ = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
    marker_rx_t_ = wallNow();
    have_marker_ = true;
  }

  void controlLoop()
  {
    const auto period = duration_cast<steady_clock::duration>(duration<double>(1.0 / rate_hz_));
    auto next = steady_clock::now();
    auto last = next;
    size_t n = 0;
    while (running_ && rclcpp::ok()) {
      next += period;
      const auto now_steady = steady_clock::now();
      const double cycle_s = duration<double>(now_steady - last).count();
      last = now_steady;
      cycle(cycle_s);
      if (++n % 200 == 0) {
        log_.flush();  // only this thread touches the log
      }
      std::this_thread::sleep_until(next);
      if (steady_clock::now() > next + period) {
        next = steady_clock::now();  // overran badly: resync instead of bursting
        std::lock_guard<std::mutex> lk(mu_);
        stats_.overruns++;
      }
    }
    log_.flush();
  }

  void cycle(double cycle_s)
  {
    const double t_cycle = wallNow();
    const double dt = 1.0 / rate_hz_;

    // Operator requests: applied here, in the control thread, so the
    // supervisor is only ever touched by one thread.
    const int e = req_engage_.exchange(-1);
    if (e >= 0) {sup_->requestEngage(e == 1);}
    const int s = req_estop_.exchange(-1);
    if (s >= 0) {sup_->setEstop(s == 1);}
    if (req_reset_.exchange(false)) {sup_->requestReset();}

    teleop_ik::CycleInput in;
    Eigen::Vector3d marker;
    double cap_t, rx_t;
    {
      std::lock_guard<std::mutex> lk(mu_);
      in.have_joints = have_joints_;
      in.joints_age = have_joints_ ? t_cycle - joints_rx_t_ : 1e9;
      in.marker_age = have_marker_ ? t_cycle - marker_rx_t_ : 1e9;
      in.q_measured = q_measured_;
      marker = marker_;
      cap_t = marker_capture_t_;
      rx_t = marker_rx_t_;
    }
    in.now = t_cycle;

    // IK proposal, only meaningful while active. The proposal is the joint
    // solution for the current hand target, not a one-cycle step: the
    // supervisor brakes against the distance to its target, and a one-step
    // proposal (a few mrad away) made it cap speed at sqrt(2 a v dt).
    const teleop_ik::Mode before = sup_->mode();
    bool clamped = false, have_target = false;
    teleop_ik::IkResult diag;
    double ik_s = 0.0;
    Eigen::Isometry3d target = anchor_;
    if (before == teleop_ik::Mode::kActive && in.marker_age <= stale_s_) {
      const Eigen::Vector3d p = anchor_.translation() + scale_ * (map_ * (marker - marker_origin_));
      const Eigen::Vector3d pc = p.cwiseMax(ws_min_).cwiseMin(ws_max_);
      clamped = (pc - p).norm() > 1e-9;
      target.translation() = pc;
      have_target = true;
      const auto t0 = steady_clock::now();
      diag = teleop_ik::solve(target, sup_->commanded(), limits_, opt_);
      in.q_proposed = diag.q;
      ik_s = duration<double>(steady_clock::now() - t0).count();
    } else {
      in.q_proposed = before == teleop_ik::Mode::kIdle ? in.q_measured : sup_->commanded();
    }

    const teleop_ik::CycleOutput out = sup_->step(in, dt);
    if (!have_target) {
      target = teleop_ik::forwardKinematics(out.q_cmd);  // no target this cycle: log the hand itself
    }

    if (out.transition) {
      const bool engaged = out.mode == teleop_ik::Mode::kActive && out.reason == "engaged";
      const bool resumed = out.mode == teleop_ik::Mode::kActive && out.reason == "marker back";
      if (engaged || resumed) {
        // Anchor where the hand is now, so neither engage nor a dropout
        // recovery makes the target jump.
        anchor_ = teleop_ik::forwardKinematics(out.q_cmd);
        marker_origin_ = marker;
      }
      publishState(out, t_cycle);
    } else if (++state_ticks_ % static_cast<int>(rate_hz_ / 5) == 0) {
      publishState(out, t_cycle);
    }

    double t_pub = 0.0;
    if (out.publish) {
      sensor_msgs::msg::JointState cmd;
      cmd.header.stamp = rclcpp::Time(static_cast<int64_t>(t_cycle * 1e9));
      cmd.name = joint_names_;
      cmd.position.assign(out.q_cmd.data(), out.q_cmd.data() + teleop_ik::kJoints);
      cmd_pub_->publish(cmd);
      t_pub = wallNow();
    }

    const Eigen::Vector3d hand = teleop_ik::forwardKinematics(out.q_cmd).translation();
    const bool stale = in.marker_age > stale_s_;
    {
      std::lock_guard<std::mutex> lk(mu_);
      stats_.cycles.push_back(cycle_s);
      if (ik_s > 0) {
        stats_.ik.push_back(ik_s);
        stats_.track.push_back((hand - target.translation()).norm());
      }
      stats_.stale += stale && out.mode != teleop_ik::Mode::kIdle;
      stats_.clamped += clamped;
      mode_ = out.mode;
      reason_ = out.reason;
    }
    if (out.mode == teleop_ik::Mode::kIdle && !out.publish) {
      return;  // nothing happening: keep the log to active periods
    }
    char line[640];
    std::snprintf(line, sizeof(line),
      "%.6f,%.6f,%.6f,1,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.1f,%.6f,%.5f,%.5f,%s,%.1f\n",
      t_cycle, cap_t, rx_t, stale, clamped,
      target.translation().x(), target.translation().y(), target.translation().z(),
      hand.x(), hand.y(), hand.z(), ik_s * 1e6, t_pub, diag.min_singular_value, diag.damping,
      teleop_ik::modeName(out.mode), std::min(in.joints_age * 1000.0, 99999.0));
    log_ << line;
  }

  void publishState(const teleop_ik::CycleOutput & out, double t)
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "{\"t\": %.3f, \"mode\": \"%s\", \"reason\": \"%s\", \"transition\": %s}",
      t, teleop_ik::modeName(out.mode), out.reason.c_str(), out.transition ? "true" : "false");
    std_msgs::msg::String m;
    m.data = buf;
    state_pub_->publish(m);
    if (out.transition) {
      RCLCPP_INFO(get_logger(), "%s", buf);
    }
  }

  void report()
  {
    Stats s;
    bool joints, marker;
    teleop_ik::Mode mode;
    std::string reason;
    {
      std::lock_guard<std::mutex> lk(mu_);
      std::swap(s, stats_);
      joints = have_joints_ && wallNow() - joints_rx_t_ < 0.5;
      marker = have_marker_ && wallNow() - marker_rx_t_ < stale_s_;
      mode = mode_;
      reason = reason_;
    }
    if (mode == teleop_ik::Mode::kIdle) {
      RCLCPP_INFO(get_logger(), "IDLE (%s). joints %s, marker %s", reason.c_str(),
        joints ? "ok" : "missing", marker ? "visible" : "not visible");
      return;
    }
    RCLCPP_INFO(get_logger(), "%s (%s) | cycles %zu | period %s | IK %s | tracking %s | stale %zu clamped %zu overruns %zu",
      teleop_ik::modeName(mode), reason.c_str(), s.cycles.size(), pct(s.cycles, 1e3, "ms").c_str(), pct(s.ik, 1e6, "us").c_str(),
      pct(s.track, 1e3, "mm").c_str(), s.stale, s.clamped, s.overruns);
  }

  struct Stats
  {
    std::vector<double> cycles, ik, track;
    size_t stale = 0, clamped = 0, overruns = 0;
  };

  double rate_hz_, scale_, stale_s_;
  Eigen::Vector3d ws_min_, ws_max_;
  Eigen::Matrix3d map_;
  teleop_ik::JointLimits limits_;
  teleop_ik::IkOptions opt_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<teleop_ik::SafetySupervisor> sup_;
  std::atomic<int> req_engage_{-1}, req_estop_{-1};
  std::atomic<bool> req_reset_{false};
  int state_ticks_ = 0;
  // Control-thread only:
  Eigen::Vector3d marker_origin_ = Eigen::Vector3d::Zero();
  Eigen::Isometry3d anchor_ = Eigen::Isometry3d::Identity();

  std::mutex mu_;  // guards everything below
  Vec7 q_measured_ = Vec7::Zero();
  bool have_joints_ = false, have_marker_ = false;
  Eigen::Vector3d marker_ = Eigen::Vector3d::Zero();
  double marker_capture_t_ = 0.0, marker_rx_t_ = 0.0, joints_rx_t_ = 0.0;
  teleop_ik::Mode mode_ = teleop_ik::Mode::kIdle;
  std::string reason_ = "start";
  Stats stats_;

  std::ofstream log_;
  std::atomic<bool> running_{true};
  std::thread loop_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_sub_, estop_sub_, reset_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
