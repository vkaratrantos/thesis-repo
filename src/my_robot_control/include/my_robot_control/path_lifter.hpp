// Turn a reduced (wrist-centre) path back into a full 7-DOF trajectory, then
// check it against the real robot.
//
// The reduced planner only ever decides where the wrist centre goes. Recovering
// the last three joints is a closed-form step, not a search, because the wrist
// is spherical: pick the tool orientation you want and SrsKinematics::wristIk
// inverts the ZYZ set directly.
//
// The orientation to pick is not fully determined, and that is the point. The
// carry constraint fixes the tool axis to vertical but leaves roll about it
// free. So the lifter takes a roll angle at each end and interpolates between
// them, which also decides where the TCP actually ends up: with the tool
// upright the wrist-centre-to-flange offset is horizontal, so
//
//     tcp = W + flange_offset * (cos phi, sin phi, 0)
//
// and phi is the roll. Choosing phi at the goal is how the lifted path is made
// to land on the requested TCP position.
//
// VALIDATION IS NOT OPTIONAL
// -------------------------
// The blob is conservative, so a reduced plan is collision free by
// construction -- but only for the wrist assembly it was swept over. Any plan
// that will actually be executed must still be replayed against the full model
// and the live scene, with the carried tube attached. validate() is that
// replay. It is cheap relative to planning and it is the thing that makes the
// whole approach safe to trust.

#pragma once

#include "my_robot_control/srs_kinematics.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace my_robot_control
{

class PathLifter
{
public:
  using JointVector = Eigen::Matrix<double, 7, 1>;

  // `full_model` must be the real 7-DOF model. `tcp_link` is the link the task
  // treats as the tool frame -- link7, which is what getEndEffectorLink()
  // reports.
  PathLifter(const moveit::core::RobotModelConstPtr & full_model,
             const std::string & tcp_link = "link7");

  struct Result
  {
    bool success{ false };
    std::vector<JointVector> joints;
    double max_tilt{ 0.0 };          // worst tool-axis deviation from vertical, rad
    std::size_t first_failure{ 0 };  // waypoint index if success is false
    std::string message;
  };

  // Lift a wrist-centre path. `roll_start` and `roll_end` are the free roll at
  // each end; the lifter interpolates linearly along the shorter arc between
  // them. Fails if any waypoint has no wrist branch within joint limits.
  Result lift(const std::vector<Eigen::Vector4d> & wrist_path,
              double roll_start,
              double roll_end) const;

  // Replay a lifted path against the live scene and the full robot. `state`
  // supplies everything outside the arm -- gripper joints, and any attached
  // objects such as the carried tube.
  //
  // Returns false and sets `first_bad` on the first colliding waypoint.
  bool validate(const planning_scene::PlanningScene & scene,
                const moveit::core::RobotState & reference_state,
                const std::vector<JointVector> & joints,
                std::size_t & first_bad) const;

  // The roll implied by an actual robot state: the direction of the
  // wrist-centre-to-flange offset, projected into the horizontal plane. Used to
  // start the interpolation from wherever the arm already is.
  double rollOf(const moveit::core::RobotState & state) const;

  // Unit vector from the wrist centre to the flange, when the tool is upright
  // and rolled by `phi`.
  //
  // It is (sin phi, -cos phi, 0), NOT (cos phi, sin phi, 0). The base upright
  // orientation q_upright = RPY(0, -pi/2, pi/2) maps the tool's z axis -- the
  // axis the flange offset lies along -- onto world -y, so the circle is a
  // quarter turn out of phase with the naive guess.
  //
  // Getting this wrong is silent when phi is only ever swept over a full
  // circle, because the SET of wrist centres comes out the same. It only bites
  // when a specific phi has to correspond to a specific wrist centre, which is
  // exactly what a planner carrying phi as a state dimension needs.
  static Eigen::Vector3d flangeDirection(double phi);

  // The roll that puts the TCP at `tcp_position` when the wrist centre is at W.
  // Inverse of flangeDirection().
  static double rollFor(const Eigen::Vector3d & wrist_centre,
                        const Eigen::Vector3d & tcp_position);

  // |joint7's origin offset|: wrist centre to flange, 0.066 m on this arm.
  double flangeOffset() const { return flange_offset_; }

  const SrsKinematics & kinematics() const { return *kin_; }

  // The upright reference orientation, RPY(0, -pi/2, pi/2) from
  // simple_move.cpp, rolled by `phi` about the world vertical.
  static Eigen::Matrix3d uprightOrientation(double phi);

private:
  moveit::core::RobotModelConstPtr model_;
  std::string tcp_link_;
  std::unique_ptr<SrsKinematics> kin_;
  double flange_offset_{ 0.0 };
};

}  // namespace my_robot_control
