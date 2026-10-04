// Compares my forward kinematics against Isaac Sim's true hand pose.
//
// Isaac publishes, from the same physics step and with the same wall-clock
// stamp, the joint angles (/isaac_joint_states_fast) and the transform
// panda_link0 -> panda_hand (/isaac_ee_tf). For each matched pair this node
// runs FK on the joint angles and reports position and rotation error.

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "teleop_ik/franka_kinematics.hpp"

using teleop_ik::Vec7;

class FkCheck : public rclcpp::Node
{
public:
  FkCheck()
  : Node("fk_check")
  {
    auto qos = rclcpp::SensorDataQoS();
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states_fast", qos, [this](sensor_msgs::msg::JointState::SharedPtr m) { onJoints(*m); });
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/isaac_ee_tf", qos, [this](tf2_msgs::msg::TFMessage::SharedPtr m) { onTf(*m); });
    timer_ = create_wall_timer(std::chrono::seconds(2), [this] { report(); });
    RCLCPP_INFO(get_logger(), "matching /isaac_joint_states_fast with /isaac_ee_tf by stamp");
  }

private:
  static int64_t ns(const builtin_interfaces::msg::Time & t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

  void onJoints(const sensor_msgs::msg::JointState & m)
  {
    Vec7 q;
    for (int i = 0; i < teleop_ik::kJoints; ++i) {
      const std::string name = "panda_joint" + std::to_string(i + 1);
      auto it = std::find(m.name.begin(), m.name.end(), name);
      if (it == m.name.end()) {return;}
      q(i) = m.position[std::distance(m.name.begin(), it)];
    }
    joints_[ns(m.header.stamp)] = q;
    while (joints_.size() > 200) {joints_.erase(joints_.begin());}
  }

  void onTf(const tf2_msgs::msg::TFMessage & m)
  {
    for (const auto & t : m.transforms) {
      if (t.child_frame_id != "panda_hand") {continue;}
      const int64_t stamp = ns(t.header.stamp);
      auto it = joints_.lower_bound(stamp - 500000);  // match within 0.5 ms
      if (it == joints_.end() || std::llabs(it->first - stamp) > 500000) {++unmatched_; continue;}
      const Eigen::Isometry3d fk = teleop_ik::forwardKinematics(it->second);
      const Eigen::Vector3d p(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
      const Eigen::Quaterniond qi(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
      pos_err_mm_.push_back((fk.translation() - p).norm() * 1000.0);
      rot_err_deg_.push_back(teleop_ik::rotationError(qi.toRotationMatrix(), fk.linear()).norm() * 180.0 / M_PI);
      if (pos_err_mm_.size() > 5000) {pos_err_mm_.pop_front(); rot_err_deg_.pop_front();}
      last_isaac_ = p;
      last_fk_ = fk.translation();
    }
  }

  static std::string stats(std::deque<double> d)
  {
    if (d.empty()) {return "n/a";}
    std::vector<double> v(d.begin(), d.end());
    std::sort(v.begin(), v.end());
    char buf[96];
    std::snprintf(buf, sizeof(buf), "p50 %.3f p99 %.3f max %.3f", v[v.size() / 2], v[v.size() * 99 / 100], v.back());
    return buf;
  }

  void report()
  {
    RCLCPP_INFO(get_logger(), "pairs %zu unmatched %zu | position error mm: %s | rotation error deg: %s",
      pos_err_mm_.size(), unmatched_, stats(pos_err_mm_).c_str(), stats(rot_err_deg_).c_str());
    if (!pos_err_mm_.empty()) {
      RCLCPP_INFO(get_logger(), "  last hand: isaac (%.4f %.4f %.4f)  fk (%.4f %.4f %.4f)",
        last_isaac_.x(), last_isaac_.y(), last_isaac_.z(), last_fk_.x(), last_fk_.y(), last_fk_.z());
    }
  }

  std::map<int64_t, Vec7> joints_;
  std::deque<double> pos_err_mm_, rot_err_deg_;
  size_t unmatched_ = 0;
  Eigen::Vector3d last_isaac_ = Eigen::Vector3d::Zero(), last_fk_ = Eigen::Vector3d::Zero();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkCheck>());
  rclcpp::shutdown();
  return 0;
}
