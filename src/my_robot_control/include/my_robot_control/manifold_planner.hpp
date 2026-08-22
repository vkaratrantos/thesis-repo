// Exact constrained planning on the upright manifold, in 5 dimensions.
//
// WHY THIS EXISTS
// ---------------
// The swept-volume approach in reduced_planner.hpp scored 0/30 on real task
// transits. The reason is geometric, not a bug: the gripper plus tube reaches
// 201 mm from the wrist centre on an arm with 243 mm of reach, so the volume
// swept over all admissible rolls is nearly as wide as the arm. Parked over the
// tube rack it always contains a neighbouring tube, and every configuration is
// rejected. On this robot the surrogate is too conservative to be usable.
//
// THE FIX
// -------
// Stop sweeping the roll and PLAN it. The insight is that the roll was never
// genuinely unknown -- it was only unknown to the 4-DOF planner. Add it as a
// fifth search dimension and the whole robot becomes determined:
//
//     q1..q4        ->  wrist centre                     (SRS decomposition)
//     phi           ->  tool orientation = upright(phi)
//     q5,q6,q7      ->  closed-form ZYZ inversion of that orientation
//
// So every point of the 5-D search space maps to a complete 7-DOF
// configuration. That buys three things at once:
//
//   * Collision checking uses the REAL gripper and the REAL tube. No swept
//     volume, no conservatism, nothing engulfing the rack.
//   * The upright constraint holds EXACTLY, by construction, at every sample --
//     not approximately via rejection sampling or numerical projection. The
//     measured tool tilt is 0.
//   * The search is still 5-dimensional rather than 7, which was the whole
//     point of the exercise.
//
// This is planning directly on the constraint manifold using an exact analytic
// chart. MoveIt's ConstrainedPlanningStateSpace attempts the same thing
// numerically by projection; on this arm that path succeeds 60% of the time and
// takes ~10 s, against 0.66 s unconstrained.
//
// THE WRIST BRANCH
// ----------------
// A ZYZ inversion has two solutions, (q5, q6, q7) and (q5+pi, -q6, q7+pi).
// Letting the checker pick per-state would break the state->configuration map:
// two nearby states could land on different branches, and the joint path would
// jump between them mid-edge.
//
// Instead the branch is chosen ONCE per plan, from the start state, and held
// fixed for the whole search. That keeps the map a genuine continuous function,
// and it guarantees the plan begins at the configuration the arm is actually
// in. If a plan fails, retrying on the other branch is a legitimate second
// attempt.

#pragma once

#include "my_robot_control/path_lifter.hpp"
#include "my_robot_control/srs_kinematics.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace my_robot_control
{

class ManifoldPlanner
{
public:
  using JointVector = Eigen::Matrix<double, 7, 1>;

  struct Options
  {
    // The FULL 7-DOF model -- this planner checks the real robot.
    moveit::core::RobotModelConstPtr model;
    std::string tcp_link{ "link7" };

    // Lock joint3 (the arm angle) to its start value, giving a 4-D search
    // instead of 5-D. Costs ~5% of the wrist-centre workspace, measured.
    bool lock_arm_angle{ false };

    double planning_time{ 2.0 };
    double longest_valid_segment_fraction{ 0.005 };
    int arm_angle_samples{ 12 };
    int roll_samples{ 24 };

    // Goal tolerances, matching what simple_move.cpp configures on the
    // MoveGroupInterface (setGoalPositionTolerance / setGoalOrientationTolerance).
    //
    // These are not a fudge -- they are parity. The constrained 7-DOF pipeline
    // is allowed to land 5 mm and 0.02 rad off the requested pose, and it uses
    // that latitude constantly: measured on the live task, a goal that has no
    // collision-free configuration at the EXACT pose frequently has one a few
    // millimetres away. Demanding the exact pose while the baseline gets a
    // tolerance makes the comparison meaningless and the planner needlessly
    // brittle.
    double goal_position_tolerance{ 0.005 };
    double goal_roll_tolerance{ 0.02 };

    // When the caller supplies a preferred goal configuration, goal candidates
    // within this distance of it (max per-joint, radians) are preferred.
    //
    // This matters more than it sounds. A pose has many configurations, and
    // arriving at the RIGHT pose in the WRONG one is a real failure mode: the
    // caller usually plans a short Cartesian move immediately afterwards, and
    // that only works from a configuration close to the one it was designed
    // around. Reaching the standoff in a flipped elbow leaves the arm unable to
    // descend.
    //
    // It is a PREFERENCE, not a filter. Enforcing it as a hard cut-off is worse
    // than useless: measured on the live task it discarded all 14 goal
    // candidates, the planner reported no goal configuration, and the caller
    // fell back to the constrained 7-DOF pipeline -- which plans to the pose
    // with no hint at all and lands wherever it likes. A preference that
    // eliminates the planner enforcing it just hands the decision to something
    // with no preference. See goal_hint_fallback_count.
    double goal_hint_radius{ 0.12 };

    // If nothing falls within goal_hint_radius, keep this many of the closest
    // candidates instead of giving up.
    std::size_t goal_hint_fallback_count{ 8 };

    // Reject goal configurations sitting closer than this to any joint stop,
    // in radians.
    //
    // A configuration can be collision-free, within bounds, and at exactly the
    // right pose while still being useless: measured, the arm reached a tube
    // standoff with joint5 0.005 rad from its 2.967 limit, and the descent that
    // followed stalled at 11% because every waypoint below it needed a 2.8 rad
    // flip to the mirror wrist branch. A goal with no room left to move is not
    // a goal worth reaching.
    double goal_limit_headroom{ 0.15 };
    bool simplify{ true };
    unsigned rng_seed{ 0 };
  };

  struct Result
  {
    bool success{ false };
    std::vector<JointVector> path;      // complete 7-DOF waypoints
    double planning_time{ 0.0 };
    std::size_t states_checked{ 0 };
    std::size_t goal_states{ 0 };
    double max_tilt{ 0.0 };             // should be ~0 by construction
    double max_joint_step{ 0.0 };       // largest joint jump between waypoints
    int branch_used{ 0 };
    std::string message;
  };

  ManifoldPlanner();
  ~ManifoldPlanner();

  ManifoldPlanner(const ManifoldPlanner &) = delete;
  ManifoldPlanner & operator=(const ManifoldPlanner &) = delete;

  void initialize(const Options & options);

  // The scene to plan against, and the state supplying everything outside the
  // arm -- gripper joints and any attached bodies such as the carried tube.
  // Unlike the swept-volume planner, the payload must be attached here: this
  // one checks the real geometry, so the tube has to be present.
  void setScene(const planning_scene::PlanningSceneConstPtr & scene,
                const moveit::core::RobotState & reference_state);

  // Plan to a TCP position with the tool upright. Tries the start state's wrist
  // branch first, then the other one.
  //
  // `required_roll`, when given, pins the roll at the goal instead of leaving it
  // free. That matters more than it looks: the carried tube hangs 201 mm off the
  // TCP, so the roll decides where the TUBE ends up, not just how the wrist is
  // twisted. Callers that specify a full target pose -- which is every call site
  // in simple_move.cpp -- must pin it, or the tube lands somewhere else on the
  // circle and the grasp or pour misses.
  //
  // Leaving it null is right only when the goal genuinely is "put the TCP here,
  // any twist will do".
  Result planToToolPose(const JointVector & start_q,
                        const Eigen::Vector3d & tcp_position,
                        const double * required_roll = nullptr,
                        const JointVector * prefer = nullptr);

  // Plan to an explicit goal CONFIGURATION on the manifold.
  //
  // Unambiguous by construction: no pose, no roll, no goal tolerance, one goal
  // state. Use this whenever the caller already knows which configuration it
  // needs -- approachTarget derives the standoff configuration from the grasp
  // configuration precisely so the straight-line descent afterwards is a short
  // joint move, and any other configuration at the same pose can leave the arm
  // unable to descend.
  //
  // Fails if `goal_q` is not on the upright manifold.
  Result planToConfiguration(const JointVector & start_q, const JointVector & goal_q);

  // The roll implied by a target orientation, i.e. the inverse of
  // uprightOrientation(). Returns false if the orientation is not upright to
  // within `tol` -- planning to a non-upright goal on the upright manifold is
  // impossible, and failing loudly beats searching forever.
  static bool rollOfOrientation(const Eigen::Matrix3d & R, double & roll,
                                double tol = 0.05);

  // Is this full configuration collision free on the real model?
  bool isValid(const JointVector & q) const;

  // Smallest distance from any joint of `q` to its own nearest limit, radians.
  double limitHeadroom(const JointVector & q) const;

  // Map a search state to a full configuration on the given branch.
  bool stateToConfig(const Eigen::Vector4d & arm, double phi, int branch,
                     JointVector & out) const;

  const SrsKinematics & kinematics() const { return *kin_; }
  double flangeOffset() const { return flange_offset_; }

  // Needed by the motion validator, which lives in the .cpp and must rebuild a
  // configuration from a search state.
  bool lockArmAngle() const { return opt_.lock_arm_angle; }
  double lockedArmAngle() const { return locked_q3_; }
  int branch() const { return branch_; }

  // Unwrap the wrist joints of `q` to be continuous with `prev`, then report
  // whether they still satisfy the joint limits. False means the continuous
  // motion would need a wrist rotation the joint cannot perform.
  bool unwrapWrist(const JointVector & prev, JointVector & q) const;

private:
  Result planOnBranch(const JointVector & start_q, const Eigen::Vector3d & tcp,
                      int branch, const double * required_roll,
                      const JointVector * prefer);

  struct Impl;
  std::unique_ptr<Impl> impl_;

  Options opt_;
  std::unique_ptr<SrsKinematics> kin_;
  planning_scene::PlanningSceneConstPtr scene_;
  double flange_offset_{ 0.0 };
  double locked_q3_{ 0.0 };
  int branch_{ 0 };
};

}  // namespace my_robot_control
