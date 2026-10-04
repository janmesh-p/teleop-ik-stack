// Marker-driven teleoperation of the Franka in Isaac Sim.
//
// Clutch model: publish true on /teleop/engage and the marker's current
// position becomes the zero point and the hand's current pose the anchor.
// Marker displacement (camera frame) maps to hand displacement (robot base
// frame); hand orientation is held at the anchor. Each control cycle runs one
// velocity-limited damped least-squares IK step from the last commanded
// joints and publishes the result.
//
// The control loop runs on its own thread with fixed deadlines, not on the
// ROS executor, so callback load cannot stretch the cycle.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "teleop_ik/dls_ik.hpp"

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
      "/teleop/engage", 10, [this](std_msgs::msg::Bool::SharedPtr m) { onEngage(m->data); });

    log_.open(log_path);
    log_ << "t_cycle,t_marker_capture,t_marker_rx,engaged,stale,clamped,"
            "target_x,target_y,target_z,hand_x,hand_y,hand_z,ik_us,t_publish,min_sv,damping\n";

    report_timer_ = create_wall_timer(seconds(2), [this] { report(); });
    loop_ = std::thread([this] { controlLoop(); });
    RCLCPP_INFO(get_logger(), "teleop at %.0f Hz, scale %.2f, log %s. Publish true on /teleop/engage to start.",
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

  void onEngage(bool on)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (on && !engaged_) {
      if (!have_joints_ || !have_marker_ || wallNow() - marker_rx_t_ > stale_s_) {
        RCLCPP_WARN(get_logger(), "engage refused: need fresh joint states and a visible marker");
        return;
      }
      q_cmd_ = q_measured_;  // resync to where the arm actually is
      anchor_ = teleop_ik::forwardKinematics(q_cmd_);
      marker_origin_ = marker_;
      last_target_p_ = anchor_.translation();
      engaged_ = true;
      RCLCPP_INFO(get_logger(), "engaged at hand (%.3f %.3f %.3f)",
        anchor_.translation().x(), anchor_.translation().y(), anchor_.translation().z());
    } else if (!on && engaged_) {
      engaged_ = false;
      RCLCPP_INFO(get_logger(), "disengaged, arm holds its last command");
    }
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
    Eigen::Isometry3d target;
    Vec7 q;
    double cap_t = 0.0, rx_t = 0.0;
    bool stale = false, clamped = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!engaged_) {
        return;
      }
      stale = t_cycle - marker_rx_t_ > stale_s_;
      if (!stale) {
        const Eigen::Vector3d delta = scale_ * (map_ * (marker_ - marker_origin_));
        const Eigen::Vector3d p = anchor_.translation() + delta;
        const Eigen::Vector3d pc = p.cwiseMax(ws_min_).cwiseMin(ws_max_);
        clamped = (pc - p).norm() > 1e-9;
        last_target_p_ = pc;
      }
      // Stale marker: keep the last target, the arm settles there.
      target = anchor_;
      target.translation() = last_target_p_;
      q = q_cmd_;
      cap_t = marker_capture_t_;
      rx_t = marker_rx_t_;
    }

    teleop_ik::IkResult diag;
    const auto t0 = steady_clock::now();
    const Vec7 q_next = teleop_ik::velocityLimitedStep(target, q, dt, limits_, opt_, &diag);
    const double ik_s = duration<double>(steady_clock::now() - t0).count();

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = rclcpp::Time(static_cast<int64_t>(t_cycle * 1e9));
    cmd.name = joint_names_;
    cmd.position.assign(q_next.data(), q_next.data() + teleop_ik::kJoints);
    cmd_pub_->publish(cmd);
    const double t_pub = wallNow();

    const Eigen::Vector3d hand = teleop_ik::forwardKinematics(q_next).translation();
    {
      std::lock_guard<std::mutex> lk(mu_);
      q_cmd_ = q_next;
      stats_.cycles.push_back(cycle_s);
      stats_.ik.push_back(ik_s);
      stats_.track.push_back((hand - target.translation()).norm());
      stats_.stale += stale;
      stats_.clamped += clamped;
    }
    char line[512];
    std::snprintf(line, sizeof(line),
      "%.6f,%.6f,%.6f,1,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.1f,%.6f,%.5f,%.5f\n",
      t_cycle, cap_t, rx_t, stale, clamped,
      target.translation().x(), target.translation().y(), target.translation().z(),
      hand.x(), hand.y(), hand.z(), ik_s * 1e6, t_pub, diag.min_singular_value, diag.damping);
    log_ << line;
  }

  void report()
  {
    Stats s;
    bool engaged, joints, marker;
    {
      std::lock_guard<std::mutex> lk(mu_);
      std::swap(s, stats_);
      engaged = engaged_;
      joints = have_joints_;
      marker = have_marker_ && wallNow() - marker_rx_t_ < stale_s_;
    }
    if (!engaged) {
      RCLCPP_INFO(get_logger(), "idle (not engaged). joints %s, marker %s",
        joints ? "ok" : "missing", marker ? "visible" : "not visible");
      return;
    }
    RCLCPP_INFO(get_logger(), "cycles %zu | period %s | IK %s | tracking %s | stale %zu clamped %zu overruns %zu",
      s.cycles.size(), pct(s.cycles, 1e3, "ms").c_str(), pct(s.ik, 1e6, "us").c_str(),
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

  std::mutex mu_;
  Vec7 q_measured_ = Vec7::Zero(), q_cmd_ = Vec7::Zero();
  bool have_joints_ = false, have_marker_ = false, engaged_ = false;
  Eigen::Vector3d marker_ = Eigen::Vector3d::Zero(), marker_origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_target_p_ = Eigen::Vector3d::Zero();
  double marker_capture_t_ = 0.0, marker_rx_t_ = 0.0;
  Eigen::Isometry3d anchor_ = Eigen::Isometry3d::Identity();
  Stats stats_;

  std::ofstream log_;
  std::atomic<bool> running_{true};
  std::thread loop_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_sub_;
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
