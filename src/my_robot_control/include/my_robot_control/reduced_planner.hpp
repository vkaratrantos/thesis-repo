// Free-space planner for the reduced (3- or 4-DOF) positioning arm.
//
// WHAT IT DOES
// ------------
// Plans the WRIST CENTRE from A to B, using OMPL over joints 1-4 (or 1,2,4)
// of a stripped-down robot model whose only distal geometry is the swept
// wrist volume produced by tools/wrist_blob_gen.py.
//
// This is the professor's suggestion made concrete: keep the existing planner
// (RRTConnect, same as ompl_planning.yaml), shrink the search space from 7
// dimensions to 4 or 3, and pay for it with a conservative collision body in
// place of the wrist.
//
// WHAT IT DELIBERATELY DOES NOT DO
// --------------------------------
// It plans positions, not poses. Orientation is recovered afterwards, in
// closed form, by SrsKinematics::lift() -- that is the whole point of the S-R-S
// decomposition. And because the blob is conservative, a plan that comes out of
// here still has to be validated against the real 7-DOF model before execution.
// Anything this planner accepts is genuinely collision free; things it rejects
// are not necessarily in collision.
//
// SCOPE
// -----
// Transit moves only. The blob contains the carried tube swept around a ring,
// so it reaches ~154 mm below the wrist centre and reports a table collision at
// grasp height. Approach, grasp and retreat stay on the existing Cartesian
// interpolation path, which never used OMPL anyway.

#pragma once

#include "my_robot_control/srs_kinematics.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace my_robot_control
{

class ReducedPlanner
{
public:
  struct Options
  {
    std::string urdf_path;                            // generated/reduced_arm.urdf
    std::string srdf_path;                            // generated/reduced_arm.srdf
    std::string group{ "arm_positioning" };           // or "arm_positioning_3dof"
    std::string wrist_centre_link{ "wrist_center" };
    std::string blob_link{ "wrist_blob" };
    std::string align_joint_x{ "blob_align_x" };
    std::string align_joint_y{ "blob_align_y" };

    double planning_time{ 2.0 };
    // Matches longest_valid_segment_fraction in ompl_planning.yaml, so the two
    // pipelines check motions at the same resolution and the benchmark is fair.
    double longest_valid_segment_fraction{ 0.005 };
    // How many joint3 values to try when turning a Cartesian goal into joint
    // goals. Ignored by the 3-DOF group, which has no joint3 to sweep.
    int arm_angle_samples{ 16 };
    bool simplify{ true };
    unsigned rng_seed{ 0 };                           // 0 = leave OMPL's default
  };

  struct Result
  {
    bool success{ false };
    std::vector<Eigen::Vector4d> path;                // always full q1..q4
    double planning_time{ 0.0 };
    std::size_t states_checked{ 0 };
    std::size_t goal_states{ 0 };
    std::string message;
  };

  ReducedPlanner();
  ~ReducedPlanner();

  ReducedPlanner(const ReducedPlanner &) = delete;
  ReducedPlanner & operator=(const ReducedPlanner &) = delete;

  // Loads the reduced model and builds the OMPL setup. Throws on a malformed
  // model or an unknown group.
  void initialize(const Options & options);

  // Copies world collision objects across from the live scene.
  //
  // `ignore_ids` must include the tube currently held: it is already inside the
  // blob, and having it in the world as well would make the blob collide with
  // its own payload at every state.
  void syncWorld(const planning_scene::PlanningScene & live_scene,
                 const std::set<std::string> & ignore_ids = {});

  // Plan the wrist centre to `goal_wrist_centre`.
  Result planToWristCentre(const Eigen::Vector4d & start_q,
                           const Eigen::Vector3d & goal_wrist_centre);

  // Plan to a TCP position with the tool held upright.
  //
  // This is the goal form the task actually uses, and it is NOT a single wrist
  // centre. With the tool axis vertical, the wrist-centre-to-flange offset is
  // horizontal and its direction is the free roll, so the wrist centre lies
  // anywhere on a horizontal circle of radius `flange_offset` about the TCP
  // position. Sampling that circle is free extra goal states -- the same
  // redundancy the 7-DOF planner has to discover by searching.
  //
  // `flange_offset` is |joint7's origin offset| from the full URDF, 0.066 m.
  Result planToToolPose(const Eigen::Vector4d & start_q,
                        const Eigen::Vector3d & tcp_position,
                        double flange_offset,
                        int roll_samples = 24);

  // Plan to an explicit joint goal (skips IK; used by the benchmark).
  Result planToJointGoal(const Eigen::Vector4d & start_q, const Eigen::Vector4d & goal_q);

  // Is this q collision free in the REDUCED model? Exposed for diagnostics --
  // it is the same test the sampler uses.
  bool isValid(const Eigen::Vector4d & q) const;

  const SrsKinematics & kinematics() const { return *kin_; }
  const moveit::core::RobotModelConstPtr & model() const { return model_; }
  const planning_scene::PlanningScenePtr & scene() const { return scene_; }

  // Sets the two align joints on `state` so the blob's axis points along world
  // z. Public because the benchmark and the RViz preview need it too.
  void alignBlob(moveit::core::RobotState & state) const;

private:
  // Shared back end for all three plan* entry points: hand OMPL a start and a
  // set of goal configurations and extract the solution.
  Result solveFrom(const Eigen::Vector4d & start_q,
                   const std::vector<Eigen::Vector4d> & goal_candidates,
                   const std::string & empty_goal_message);

  struct Impl;
  std::unique_ptr<Impl> impl_;

  Options opt_;
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<SrsKinematics> kin_;

  const moveit::core::JointModelGroup * jmg_{ nullptr };
  std::vector<int> dim_to_arm_index_;   // state dim -> index into q1..q4
  double locked_q3_{ 0.0 };
};

}  // namespace my_robot_control
