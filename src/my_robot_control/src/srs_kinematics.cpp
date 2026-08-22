#include "my_robot_control/srs_kinematics.hpp"

#include <moveit/robot_state/robot_state.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace my_robot_control
{
namespace
{

constexpr double kEps = 1e-9;

Eigen::Matrix3d Rx(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d Rz(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

const char * kArmJoints[4] = { "joint1", "joint2", "joint3", "joint4" };
const char * kWristJoints[3] = { "joint5", "joint6", "joint7" };

// Where the wrist centre sits in the frame of link2, as a function of the
// elbow angle and the arm angle.
//
// Derivation, following the URDF chain from link2 outward:
//
//   S -> elbow  is  (0, -L1, 0)                     in link2's frame
//   elbow -> W  is  L2 * (-s4*c3, -c4, -s4*s3)      in link2's frame
//
// The dot product of the two unit vectors is cos(q4), which is the statement
// that |W - S| depends on the elbow alone.
Eigen::Vector3d linkTwoVector(double l1, double l2, double q3, double q4)
{
  const double s3 = std::sin(q3), c3 = std::cos(q3);
  const double s4 = std::sin(q4), c4 = std::cos(q4);
  return { -l2 * s4 * c3, -l1 - l2 * c4, -l2 * s4 * s3 };
}

}  // namespace

SrsKinematics::SrsKinematics(const moveit::core::RobotModelConstPtr & model,
                             const std::string & wrist_centre_link)
{
  if (!model)
    throw std::runtime_error("SrsKinematics: null robot model");

  for (const char * n : kArmJoints)
  {
    if (!model->hasJointModel(n))
      throw std::runtime_error(std::string("SrsKinematics: model has no joint '") + n + "'");
  }
  if (!model->hasLinkModel(wrist_centre_link))
    throw std::runtime_error("SrsKinematics: model has no link '" + wrist_centre_link + "'");

  has_wrist_joints_ = model->hasJointModel(kWristJoints[0]) &&
                      model->hasJointModel(kWristJoints[1]) &&
                      model->hasJointModel(kWristJoints[2]);

  const auto record_limits = [&](const char * name) {
    const auto & b = model->getJointModel(name)->getVariableBounds().front();
    lower_.push_back(b.position_bounded_ ? b.min_position_ : -M_PI);
    upper_.push_back(b.position_bounded_ ? b.max_position_ : M_PI);
  };
  for (const char * n : kArmJoints) record_limits(n);
  if (has_wrist_joints_)
    for (const char * n : kWristJoints) record_limits(n);

  // ---- pull the geometry out of the model -------------------------------
  //
  // The shoulder point is link1's frame origin: joint2 sits at zero offset
  // from it, and joint3's offset lies ALONG joint3's own axis, so all three
  // axes pass through that single point. link3 and link4 share an origin for
  // the same reason, and that origin is the elbow.
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const char * n : kArmJoints)
  {
    const double zero = 0.0;
    state.setJointPositions(n, &zero);
  }
  state.update();

  shoulder_ = state.getGlobalLinkTransform("link1").translation();
  const Eigen::Vector3d elbow = state.getGlobalLinkTransform("link3").translation();
  const Eigen::Vector3d wrist = state.getGlobalLinkTransform(wrist_centre_link).translation();

  l1_ = (elbow - shoulder_).norm();
  l2_ = (wrist - elbow).norm();

  // Closest the wrist centre can come to the shoulder, given the elbow limits.
  {
    const double q4_max_bend = std::max(std::abs(lower_[3]), std::abs(upper_[3]));
    const double c = std::cos(q4_max_bend);
    minReach_ = std::sqrt(std::max(l1_ * l1_ + l2_ * l2_ + 2.0 * l1_ * l2_ * c, 0.0));
  }

  // ---- self-test: analytic FK against MoveIt's ---------------------------
  //
  // The closed form below assumes this URDF's specific frame conventions. If
  // someone regenerates the URDF with different rpy offsets, the algebra goes
  // silently wrong. Fail loudly here instead.
  double worst_p = 0.0, worst_r = 0.0;
  unsigned seed = 12345;
  const auto rnd = [&seed](double lo, double hi) {
    seed = seed * 1103515245u + 12345u;
    return lo + (hi - lo) * ((seed >> 16) & 0x7fff) / double(0x7fff);
  };
  for (int i = 0; i < 512; ++i)
  {
    Eigen::Vector4d q;
    for (int k = 0; k < 4; ++k) q[k] = rnd(lower_[k], upper_[k]);

    for (int k = 0; k < 4; ++k) state.setJointPositions(kArmJoints[k], &q[k]);
    state.update();
    const Eigen::Isometry3d T = state.getGlobalLinkTransform(wrist_centre_link);

    worst_p = std::max(worst_p, (T.translation() - wristCentre(q)).norm());
    worst_r = std::max(worst_r, (T.rotation() - wristFrame(q)).cwiseAbs().maxCoeff());
  }
  self_test_error_ = worst_p;

  // Tolerance, and why it is not tighter.
  //
  // The URDF writes every right angle as "1.5708", which is 3.7e-6 rad short of
  // pi/2. The closed form uses the exact value, because that is what the
  // mechanism is -- so the two disagree by about 1.3 um at the wrist centre.
  // Checked directly: replaying the same FK with exact pi/2 in the URDF brings
  // the disagreement down to 1e-16 m, so the algebra is right and the residual
  // is entirely the URDF's rounding.
  //
  // 1.3 um is four orders of magnitude below the collision checker's resolution
  // and far below the arm's repeatability, so it is ignored. The threshold is
  // set well above it but well below anything a real frame-convention change
  // could produce, which would be centimetres or radians.
  constexpr double kMaxPositionError = 1e-4;   // m
  constexpr double kMaxRotationError = 1e-3;

  if (worst_p > kMaxPositionError || worst_r > kMaxRotationError)
  {
    std::ostringstream os;
    os << "SrsKinematics: analytic FK disagrees with the robot model "
       << "(position " << worst_p << " m, rotation " << worst_r << "). "
       << "The closed form assumes the MyArm 300 Pi frame conventions; "
       << "if the URDF changed, the derivation in this file must be redone.";
    throw std::runtime_error(os.str());
  }
}

bool SrsKinematics::inLimits(int joint_index, double q) const
{
  if (joint_index >= static_cast<int>(lower_.size())) return true;
  return q >= lower_[joint_index] - 1e-9 && q <= upper_[joint_index] + 1e-9;
}

// R_shoulder = Rz(q1) Rx(-pi/2) Rz(q2), which is the rotation of link2's frame.
// (Rx(-pi/2) Rz(q2) Rx(pi/2) == Ry(q2), so this is the ZYZ set advertised in
// the header, just written in the URDF's own basis.)
Eigen::Matrix3d SrsKinematics::shoulderRot(double q1, double q2) const
{
  return Rz(q1) * Rx(-M_PI / 2.0) * Rz(q2);
}

Eigen::Vector3d SrsKinematics::wristCentre(const Eigen::Vector4d & q) const
{
  return shoulder_ + shoulderRot(q[0], q[1]) * linkTwoVector(l1_, l2_, q[2], q[3]);
}

Eigen::Matrix3d SrsKinematics::wristFrame(const Eigen::Vector4d & q) const
{
  // link2 -> joint4 axis frame -> apply q4 -> joint5's fixed origin rotation.
  return shoulderRot(q[0], q[1]) * Rx(M_PI / 2.0) * Rz(q[2]) * Rx(M_PI / 2.0) *
         Rz(q[3]) * Rx(-M_PI / 2.0);
}

std::vector<Eigen::Vector4d> SrsKinematics::positionIk(const Eigen::Vector3d & W, double q3) const
{
  std::vector<Eigen::Vector4d> out;

  const Eigen::Vector3d t = W - shoulder_;
  const double d = t.norm();

  // ---- elbow, from the law of cosines ------------------------------------
  const double denom = 2.0 * l1_ * l2_;
  if (denom < kEps) return out;
  double c4 = (d * d - l1_ * l1_ - l2_ * l2_) / denom;
  if (c4 > 1.0 + 1e-9 || c4 < -1.0 - 1e-9) return out;   // out of reach
  c4 = std::clamp(c4, -1.0, 1.0);
  const double q4_mag = std::acos(c4);

  for (const double q4 : { q4_mag, -q4_mag })
  {
    if (!inLimits(3, q4)) continue;

    const Eigen::Vector3d v = linkTwoVector(l1_, l2_, q3, q4);

    // ---- shoulder ---------------------------------------------------------
    // Need Rz(q1) Rx(-pi/2) Rz(q2) v == t. Rz(q1) cannot change the z
    // component, so q2 is pinned by z alone:
    //
    //   w = Rx(-pi/2) Rz(q2) v   =>   w_z = -(v_x s2 + v_y c2) = t_z
    //
    // which is R*sin(q2 + phi) = -t_z with R = hypot(v_x, v_y).
    const double Rm = std::hypot(v.x(), v.y());
    if (Rm < kEps) continue;
    const double ratio = -t.z() / Rm;
    if (std::abs(ratio) > 1.0 + 1e-9) continue;

    const double phi = std::atan2(v.y(), v.x());
    const double base = std::asin(std::clamp(ratio, -1.0, 1.0));

    for (const double q2_raw : { base - phi, M_PI - base - phi })
    {
      // fold into (-pi, pi] so the limit test is meaningful
      double q2 = std::remainder(q2_raw, 2.0 * M_PI);
      if (!inLimits(1, q2)) continue;

      const Eigen::Vector3d w = Rx(-M_PI / 2.0) * Rz(q2) * v;
      if (std::hypot(w.x(), w.y()) < kEps) continue;

      double q1 = std::remainder(std::atan2(t.y(), t.x()) - std::atan2(w.y(), w.x()),
                                 2.0 * M_PI);
      if (!inLimits(0, q1)) continue;
      if (!inLimits(2, q3)) continue;

      const Eigen::Vector4d q(q1, q2, q3, q4);

      // Belt and braces: the branch algebra above has sign traps, so verify
      // rather than trust. Costs one FK per candidate.
      if ((wristCentre(q) - W).norm() > 1e-6) continue;

      out.push_back(q);
    }
  }
  return out;
}

std::vector<Eigen::Vector4d> SrsKinematics::positionIkSampled(const Eigen::Vector3d & W,
                                                              int samples) const
{
  std::vector<Eigen::Vector4d> out;
  if (samples < 1) samples = 1;

  if (samples == 1)
    return positionIk(W, 0.0);

  for (int i = 0; i < samples; ++i)
  {
    const double a = static_cast<double>(i) / (samples - 1);
    const double q3 = lower_[2] + a * (upper_[2] - lower_[2]);
    for (const auto & q : positionIk(W, q3)) out.push_back(q);
  }
  return out;
}

bool SrsKinematics::wristIkBranch(const Eigen::Vector4d & q,
                                  const Eigen::Matrix3d & R_tool_world,
                                  int branch,
                                  Eigen::Vector3d & out) const
{
  if (!has_wrist_joints_) return false;

  // R_tool_world = R_wristframe * Rz(q5) Ry(-q6) Rz(q7)
  const Eigen::Matrix3d M = wristFrame(q).transpose() * R_tool_world;

  const double c = std::clamp(M(2, 2), -1.0, 1.0);
  const double b = (branch == 0) ? std::acos(c) : -std::acos(c);
  const double sb = std::sin(b);

  double q5, q7;
  if (std::abs(sb) < 1e-7)
  {
    // Wrist singularity: joints 5 and 7 are collinear and only their sum (or
    // difference) is determined. Spend it all on joint7 and leave joint5 put.
    // Both branches coincide here.
    q5 = 0.0;
    q7 = (c > 0.0) ? std::atan2(M(1, 0), M(0, 0)) : -std::atan2(M(1, 0), M(0, 0));
  }
  else
  {
    q5 = std::atan2(M(1, 2) / sb, M(0, 2) / sb);
    q7 = std::atan2(M(2, 1) / sb, -M(2, 0) / sb);
  }
  const double q6 = -b;

  if (!inLimits(4, q5) || !inLimits(5, q6) || !inLimits(6, q7)) return false;
  out = Eigen::Vector3d(q5, q6, q7);
  return true;
}

std::vector<Eigen::Vector3d> SrsKinematics::wristIk(const Eigen::Vector4d & q,
                                                    const Eigen::Matrix3d & R_tool_world) const
{
  std::vector<Eigen::Vector3d> out;
  Eigen::Vector3d w;
  for (int branch : { 0, 1 })
  {
    if (!wristIkBranch(q, R_tool_world, branch, w)) continue;
    // At the singularity the two branches are the same solution.
    if (!out.empty() && (out.front() - w).cwiseAbs().maxCoeff() < 1e-9) break;
    out.push_back(w);
  }
  return out;
}

bool SrsKinematics::lift(const Eigen::Vector4d & q,
                         const Eigen::Matrix3d & R_tool_world,
                         const Eigen::Vector3d & seed,
                         Eigen::Matrix<double, 7, 1> & out) const
{
  const auto branches = wristIk(q, R_tool_world);
  if (branches.empty()) return false;

  const auto * best = &branches.front();
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & b : branches)
  {
    const double cost = (b - seed).cwiseAbs().sum();
    if (cost < best_cost)
    {
      best_cost = cost;
      best = &b;
    }
  }

  out.head<4>() = q;
  out.tail<3>() = *best;
  return true;
}

}  // namespace my_robot_control
