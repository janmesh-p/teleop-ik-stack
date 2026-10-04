#pragma once

#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace teleop_ik
{

constexpr int kJoints = 7;
using Vec7 = Eigen::Matrix<double, kJoints, 1>;
using Jacobian = Eigen::Matrix<double, 6, kJoints>;

struct JointLimits
{
  Vec7 lower;
  Vec7 upper;
  Vec7 max_velocity;  // rad/s
};

/// Franka Emika Panda limits from the manufacturer's datasheet.
JointLimits frankaLimits();

/// A comfortable mid-range configuration, used as the IK seed.
Vec7 frankaHome();

/// Forward kinematics of the Franka, base (panda_link0) to panda_hand.
///
/// Modified Denavit-Hartenberg parameters (Craig convention) from the
/// manufacturer: each joint applies RotX(alpha) TransX(a) RotZ(q) TransZ(d).
/// After joint 7 come the flange (0.107 m along z) and the hand frame,
/// rotated -45 degrees about z. If joint_frames is given, entry i holds the
/// frame of joint i (its z axis is the joint axis), in the base frame.
Eigen::Isometry3d forwardKinematics(
  const Vec7 & q, std::array<Eigen::Isometry3d, kJoints> * joint_frames = nullptr);

/// Geometric Jacobian of panda_hand in the base frame.
/// Rows 0-2: linear velocity, rows 3-5: angular velocity.
Jacobian jacobian(const Vec7 & q);

/// Orientation error as a rotation vector (axis * angle) in the base frame,
/// taking `current` onto `target`.
Eigen::Vector3d rotationError(const Eigen::Matrix3d & target, const Eigen::Matrix3d & current);

}  // namespace teleop_ik
