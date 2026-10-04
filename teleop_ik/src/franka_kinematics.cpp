#include "teleop_ik/franka_kinematics.hpp"

#include <cmath>

namespace teleop_ik
{
namespace
{
struct ModifiedDH
{
  double a, d, alpha;
};

// Franka Emika Panda, modified DH (Craig), joints 1 to 7.
constexpr std::array<ModifiedDH, kJoints> kDH = {{
  {0.0, 0.333, 0.0},
  {0.0, 0.0, -M_PI / 2},
  {0.0, 0.316, M_PI / 2},
  {0.0825, 0.0, M_PI / 2},
  {-0.0825, 0.384, -M_PI / 2},
  {0.0, 0.0, M_PI / 2},
  {0.088, 0.0, M_PI / 2},
}};
constexpr double kFlangeD = 0.107;
constexpr double kHandYaw = -M_PI / 4;

Eigen::Isometry3d dhTransform(const ModifiedDH & p, double q)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.rotate(Eigen::AngleAxisd(p.alpha, Eigen::Vector3d::UnitX()));
  t.translate(Eigen::Vector3d(p.a, 0.0, 0.0));
  t.rotate(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
  t.translate(Eigen::Vector3d(0.0, 0.0, p.d));
  return t;
}
}  // namespace

JointLimits frankaLimits()
{
  JointLimits l;
  l.lower << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  l.upper << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  l.max_velocity << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;
  return l;
}

Vec7 frankaHome()
{
  Vec7 q;
  q << 0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0, M_PI / 2, M_PI / 4;
  return q;
}

Eigen::Isometry3d forwardKinematics(const Vec7 & q, std::array<Eigen::Isometry3d, kJoints> * joint_frames)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int i = 0; i < kJoints; ++i) {
    t = t * dhTransform(kDH[i], q(i));
    if (joint_frames) {
      (*joint_frames)[i] = t;
    }
  }
  t.translate(Eigen::Vector3d(0.0, 0.0, kFlangeD));
  t.rotate(Eigen::AngleAxisd(kHandYaw, Eigen::Vector3d::UnitZ()));
  return t;
}

Jacobian jacobian(const Vec7 & q)
{
  std::array<Eigen::Isometry3d, kJoints> frames;
  const Eigen::Vector3d p_ee = forwardKinematics(q, &frames).translation();
  Jacobian j;
  for (int i = 0; i < kJoints; ++i) {
    const Eigen::Vector3d z = frames[i].linear().col(2);
    const Eigen::Vector3d p = frames[i].translation();
    j.block<3, 1>(0, i) = z.cross(p_ee - p);
    j.block<3, 1>(3, i) = z;
  }
  return j;
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d & target, const Eigen::Matrix3d & current)
{
  const Eigen::AngleAxisd aa(target * current.transpose());
  return aa.axis() * aa.angle();
}

}  // namespace teleop_ik
