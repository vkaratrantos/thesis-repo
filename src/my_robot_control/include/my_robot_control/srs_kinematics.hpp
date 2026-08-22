// Closed-form kinematics for the S-R-S structure of the MyArm 300 Pi.
//
// The arm decomposes exactly:
//
//   joints 1,2,3  spherical shoulder, all three axes through one point S
//   joint  4      elbow
//   joints 5,6,7  spherical wrist, all three axes through one point W
//
// (Verified numerically against the URDF: the pairwise common-normal distance
// between axes 1-2, 1-3, 2-3 is 0, and likewise for 5-6, 6-7, 5-7.)
//
// Two consequences carry the whole reduced-planning approach:
//
//   * W depends on q1..q4 ONLY. Joints 5-7 change orientation and nothing
//     else. So position planning is a 4-DOF problem, not a 7-DOF one.
//
//   * |W - S| depends on q4 ONLY, because the shoulder is spherical and can
//     only rotate the upper-arm/forearm triangle, never change its shape.
//     That makes the elbow angle a one-line law of cosines.
//
// Both the shoulder and the wrist turn out to be ZYZ Euler sets once the URDF's
// fixed rpy offsets are folded in:
//
//   R_shoulder = Rz(q1) Ry(q2) Rz(q3)
//   R_wrist    = Rz(q5) Ry(-q6) Rz(q7)
//
// which is what makes both invertible in closed form.
//
// Everything here is pure: no RobotState, no mutable members, safe to call from
// several planner threads. The RobotModel is used at construction only, to pull
// out S, the link lengths, and to self-test the analytic FK against MoveIt's.

#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>

#include <string>
#include <vector>

namespace my_robot_control
{

class SrsKinematics
{
public:
  // `model` may be either the full 7-DOF model or the reduced one -- the
  // constructor only needs joints 1-4 and a frame at the wrist centre.
  //
  // Throws std::runtime_error if the analytic FK disagrees with MoveIt's FK,
  // which is the tripwire for someone changing the URDF's frame conventions
  // out from under the closed form.
  SrsKinematics(const moveit::core::RobotModelConstPtr & model,
                const std::string & wrist_centre_link);

  // ---- forward -----------------------------------------------------------

  // Wrist-centre position in the model's root frame.
  Eigen::Vector3d wristCentre(const Eigen::Vector4d & q) const;

  // Orientation of the wrist-centre frame (its z is the joint5 axis).
  Eigen::Matrix3d wristFrame(const Eigen::Vector4d & q) const;

  // ---- inverse -----------------------------------------------------------

  // Every (q1..q4) that puts the wrist centre at W with joint3 held at `q3`.
  // Up to four: two elbow branches times two shoulder branches. Solutions
  // outside the joint limits are dropped.
  std::vector<Eigen::Vector4d> positionIk(const Eigen::Vector3d & W, double q3) const;

  // Same, sweeping joint3 -- the arm-angle redundancy -- over `samples` values
  // across its limits. This is how the reduced planner builds a goal set: one
  // Cartesian target maps to a whole family of joint configurations, and
  // handing OMPL all of them is what makes the goal easy to hit.
  std::vector<Eigen::Vector4d> positionIkSampled(const Eigen::Vector3d & W, int samples) const;

  // (q5,q6,q7) achieving the world tool orientation R, given the arm at q.
  // Two ZYZ branches; both are returned, limit-filtered.
  std::vector<Eigen::Vector3d> wristIk(const Eigen::Vector4d & q,
                                       const Eigen::Matrix3d & R_tool_world) const;

  // One specific ZYZ branch, indexed stably: 0 is the +acos(M22) solution, 1
  // the -acos one. Returns false if that branch violates the joint limits.
  //
  // wristIk() drops out-of-limit branches, which makes its indices shift from
  // state to state -- sols[0] can be the positive branch at one configuration
  // and the negative branch at a neighbouring one. Anything that needs the
  // branch to be a CONTINUOUS function of the state, such as a planner whose
  // state space includes the roll, must use this instead.
  bool wristIkBranch(const Eigen::Vector4d & q,
                     const Eigen::Matrix3d & R_tool_world,
                     int branch,
                     Eigen::Vector3d & out) const;

  // Convenience: q1..q4 plus the wrist branch closest to `seed` (for
  // continuity along a path). False if no branch is within limits.
  bool lift(const Eigen::Vector4d & q,
            const Eigen::Matrix3d & R_tool_world,
            const Eigen::Vector3d & seed,
            Eigen::Matrix<double, 7, 1> & out) const;

  // ---- geometry ----------------------------------------------------------

  const Eigen::Vector3d & shoulder() const { return shoulder_; }
  double upperArm() const { return l1_; }
  double forearm() const { return l2_; }
  double maxReach() const { return l1_ + l2_; }
  double minReach() const { return minReach_; }

  // Worst analytic-vs-MoveIt FK disagreement seen during the constructor's
  // self-test, in metres. Report it; do not ignore it.
  double selfTestError() const { return self_test_error_; }

  // Joint limits, in the order q1..q7 (q5..q7 only present on a full model).
  const std::vector<double> & lower() const { return lower_; }
  const std::vector<double> & upper() const { return upper_; }

private:
  Eigen::Matrix3d shoulderRot(double q1, double q2) const;
  bool inLimits(int joint_index, double q) const;

  Eigen::Vector3d shoulder_;
  double l1_{ 0.0 };
  double l2_{ 0.0 };
  double minReach_{ 0.0 };
  double self_test_error_{ 0.0 };
  bool has_wrist_joints_{ false };

  std::vector<double> lower_;
  std::vector<double> upper_;
};

}  // namespace my_robot_control
