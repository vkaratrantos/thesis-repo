// Head-to-head: constrained 7-DOF planning vs reduced-space planning.
//
// Needs move_group running (demo.launch.py is enough -- fake hardware is fine,
// nothing is executed). Every method sees the same start states, the same goals
// and the same collision scene.
//
//   ros2 launch robot_config demo.launch.py
//   ros2 run my_robot_control benchmark_reduced --ros-args -p gen_dir:=...
//
// THE THREE METHODS
// -----------------
//   7dof-strict   What moveFreeSpaceUpright() does today: a single pose goal at
//                 the fixed q_upright yaw, plus the upright path constraint.
//                 This is the baseline being improved on.
//
//   7dof-free     The same, but the goal is given as a fan of poses at
//                 different rolls about the vertical. Roll does not change
//                 whether the tube spills, so this freedom is legitimate -- and
//                 the reduced method gets it for free, so withholding it from
//                 the 7-DOF side would rig the comparison.
//
//   reduced       Plan the wrist centre over 3-4 joints against the swept-
//                 volume blob, then recover joints 5-7 in closed form and
//                 replay the result against the full model.
//
// WHAT IS TIMED
// -------------
// For the 7-DOF methods, both the planner's own reported time and the wall
// clock across plan() (which includes the ROS round trip). For the reduced
// method, wall clock covering goal IK, the OMPL solve, the lift AND the
// full-model validation -- everything needed before the path could be executed.
// Reporting anything less would flatter it.

#include "my_robot_control/path_lifter.hpp"
#include "my_robot_control/reduced_planner.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
  #define MRC_PLAN_TRAJ(p) ((p).trajectory)
  #define MRC_PLAN_TIME(p) ((p).planning_time)
#else
  #define MRC_PLAN_TRAJ(p) ((p).trajectory_)
  #define MRC_PLAN_TIME(p) ((p).planning_time_)
#endif

using MGI = moveit::planning_interface::MoveGroupInterface;
using namespace my_robot_control;

namespace
{

constexpr double TILT_TOLERANCE = 0.30;
constexpr double TUBE_HEIGHT = 0.13, TUBE_RADIUS = 0.012;
constexpr double TUBE_TCP_Y_OFFSET = -0.135, TUBE_TCP_Z_OFFSET = -0.030;

// Task waypoints, from simple_move.cpp's fallback constants: five tube stations
// and the mixer. Transit moves run between these.
struct Station { const char * name; double x, y; };
const Station kStations[] = {
  { "tube1", -0.17, -0.22 }, { "tube2", -0.10, -0.22 }, { "tube3", 0.00, -0.22 },
  { "tube4", 0.10, -0.22 },  { "tube5", 0.17, -0.22 },  { "mixer", -0.20, -0.11 },
};

geometry_msgs::msg::Quaternion uprightQuat(double roll)
{
  tf2::Quaternion base;
  base.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);
  tf2::Quaternion spin;
  spin.setRPY(0.0, 0.0, roll);
  return tf2::toMsg(spin * base);
}

// Mirrors uprightConstraint() in simple_move.cpp.
moveit_msgs::msg::Constraints uprightConstraint(const std::string & tcp_link,
                                                const std::string & frame)
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = tcp_link;
  ocm.header.frame_id = frame;
  ocm.orientation = uprightQuat(0.0);
  ocm.absolute_x_axis_tolerance = M_PI;             // roll about the tube: free
  ocm.absolute_y_axis_tolerance = TILT_TOLERANCE;
  ocm.absolute_z_axis_tolerance = TILT_TOLERANCE;
  ocm.weight = 1.0;
  ocm.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;

  moveit_msgs::msg::Constraints c;
  c.orientation_constraints.push_back(ocm);
  return c;
}

struct Sample
{
  bool success{ false };
  double wall{ 0.0 };
  double reported{ 0.0 };
  double path_length{ 0.0 };
  double max_tilt{ 0.0 };
  bool validated{ true };     // reduced only: did it survive the full model
};

struct Stats
{
  int runs{ 0 }, ok{ 0 }, validated{ 0 };
  std::vector<double> wall, reported, len, tilt;

  void add(const Sample & s)
  {
    ++runs;
    if (!s.success) return;
    ++ok;
    if (s.validated) ++validated;
    wall.push_back(s.wall);
    reported.push_back(s.reported);
    len.push_back(s.path_length);
    tilt.push_back(s.max_tilt);
  }
};

double median(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
double mean(const std::vector<double> & v)
{
  return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double jointPathLength(const std::vector<Eigen::Matrix<double, 7, 1>> & q)
{
  double d = 0.0;
  for (std::size_t i = 1; i < q.size(); ++i) d += (q[i] - q[i - 1]).cwiseAbs().sum();
  return d;
}

double trajPathLength(const moveit_msgs::msg::RobotTrajectory & t)
{
  const auto & pts = t.joint_trajectory.points;
  double d = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i)
    for (std::size_t k = 0; k < pts[i].positions.size(); ++k)
      d += std::abs(pts[i].positions[k] - pts[i - 1].positions[k]);
  return d;
}

double maxTilt(const moveit_msgs::msg::RobotTrajectory & t,
               moveit::core::RobotState state, const std::string & tcp_link)
{
  double worst = 0.0;
  const auto & jt = t.joint_trajectory;
  for (const auto & p : jt.points)
  {
    for (std::size_t k = 0; k < jt.joint_names.size(); ++k)
      state.setJointPositions(jt.joint_names[k], &p.positions[k]);
    state.update();
    const Eigen::Vector3d axis = state.getGlobalLinkTransform(tcp_link).rotation().col(0);
    worst = std::max(worst, std::atan2(std::hypot(axis.x(), axis.y()), axis.z()));
  }
  return worst;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "benchmark_reduced",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const std::string gen_dir =
      node->get_parameter_or<std::string>("gen_dir", "generated");
  const std::string group = node->get_parameter_or<std::string>("group", "arm_positioning");
  const int reps = node->get_parameter_or<int>("reps", 3);
  // Defaults mirror simple_move.cpp's arm_interface configuration.
  const double budget = node->get_parameter_or<double>("planning_time", 10.0);
  const int attempts = node->get_parameter_or<int>("attempts", 5);
  const double goal_pos_tol = node->get_parameter_or<double>("goal_pos_tol", 0.005);
  const double goal_ori_tol = node->get_parameter_or<double>("goal_ori_tol", 0.02);
  const int roll_fan = node->get_parameter_or<int>("roll_fan", 12);
  // Cap the query set. A failing constrained plan costs the whole budget, so
  // runtime is roughly queries * reps * 3 * planning_time. At the faithful 10 s
  // that adds up fast; capping keeps a run to minutes without touching the
  // planner settings, which are the thing that must stay faithful.
  const int max_queries = node->get_parameter_or<int>("max_queries", 0);
  const std::string csv = node->get_parameter_or<std::string>("csv", "");
  std::vector<double> heights =
      node->get_parameter_or<std::vector<double>>("heights", { 0.22, 0.24, 0.26 });

  std::thread spinner([node] { rclcpp::spin(node); });

  // ---- live scene --------------------------------------------------------
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description");
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  if (!psm->requestPlanningSceneState("/get_planning_scene"))
  {
    RCLCPP_FATAL(node->get_logger(), "cannot fetch the planning scene -- is move_group running?");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  MGI mg(node, "arm_group");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(budget);

  // Match what simple_move.cpp actually configures (lines 1646-1652). Leaving
  // MoveIt's defaults in place would hand the 7-DOF side a 1e-4 m / 1e-3 rad
  // goal tolerance and a single attempt -- far tighter and far less persistent
  // than the real pipeline, which would make the baseline look much worse than
  // it is and the comparison dishonest.
  //
  // Note that attempts > 1 means the reported time covers up to five internal
  // tries. That is the real cost of the real configuration, so it stays.
  mg.setNumPlanningAttempts(attempts);
  mg.setGoalPositionTolerance(goal_pos_tol);
  mg.setGoalOrientationTolerance(goal_ori_tol);
  const std::string tcp_link = mg.getEndEffectorLink();
  const std::string frame = mg.getPlanningFrame();

  const auto full_model = mg.getRobotModel();
  PathLifter lifter(full_model, tcp_link);

  ReducedPlanner reduced;
  ReducedPlanner::Options ropt;
  ropt.urdf_path = gen_dir + "/reduced_arm.urdf";
  ropt.srdf_path = gen_dir + "/reduced_arm.srdf";
  ropt.group = group;
  ropt.planning_time = budget;
  reduced.initialize(ropt);

  // ---- publish the scene to move_group -----------------------------------
  //
  // This has to go through move_group, not into a local copy. simple_move.cpp
  // is what normally publishes the table, tubes and mixer, and it is not
  // running here -- so move_group's world starts EMPTY. Building the scene
  // locally and leaving move_group's empty would have the 7-DOF side planning
  // through open air while the reduced side plans against real obstacles, and
  // the whole comparison would be worthless.
  //
  // Attaching the tube matters for the same reason: the benchmark is about
  // CARRYING one, and the blob already contains it, so the 7-DOF side must be
  // holding it too.
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    const auto box = [&](const std::string & id, double sx, double sy, double sz,
                         double x, double y, double z) {
      moveit_msgs::msg::CollisionObject o;
      o.id = id; o.header.frame_id = frame;
      o.primitives.resize(1);
      o.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      o.primitives[0].dimensions = { sx, sy, sz };
      o.primitive_poses.resize(1);
      o.primitive_poses[0].position.x = x;
      o.primitive_poses[0].position.y = y;
      o.primitive_poses[0].position.z = z;
      o.primitive_poses[0].orientation.w = 1.0;
      o.operation = o.ADD;
      objects.push_back(o);
    };
    const auto cyl = [&](const std::string & id, double r, double h,
                         double x, double y, double z) {
      moveit_msgs::msg::CollisionObject o;
      o.id = id; o.header.frame_id = frame;
      o.primitives.resize(1);
      o.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      o.primitives[0].dimensions = { h, r };
      o.primitive_poses.resize(1);
      o.primitive_poses[0].position.x = x;
      o.primitive_poses[0].position.y = y;
      o.primitive_poses[0].position.z = z;
      o.primitive_poses[0].orientation.w = 1.0;
      o.operation = o.ADD;
      objects.push_back(o);
    };

    box("table_base", 1.5, 1.5, 0.04, 0.0, 0.0, -0.03);
    box("obstacle_box", 0.7, 0.2, 0.05, 0.0, 0.18, 0.02);
    box("obstacle_box_2", 0.4, 0.2, 0.3, -0.6, 0.1, 0.03);
    const double tube_x[5] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
    for (int i = 0; i < 5; ++i)
      cyl("tube_" + std::to_string(i + 1), TUBE_RADIUS, TUBE_HEIGHT, tube_x[i], -0.345, 0.08);
    cyl("mixer", 0.06, 0.17, -0.32, -0.23, 0.08);
    psi.applyCollisionObjects(objects);

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = tcp_link;
    aco.object.id = "carried_tube";
    aco.object.header.frame_id = tcp_link;
    aco.object.primitives.resize(1);
    aco.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    aco.object.primitives[0].dimensions = { TUBE_HEIGHT, TUBE_RADIUS };
    aco.object.primitive_poses.resize(1);
    // In the TCP frame the tube sits 0.135 along +z, offset -0.030 along x, with
    // its long axis on the TCP's x. See wrist_blob_gen.py for how this is
    // derived from simple_move.cpp's world-frame version.
    aco.object.primitive_poses[0].position.x = TUBE_TCP_Z_OFFSET;
    aco.object.primitive_poses[0].position.z = -TUBE_TCP_Y_OFFSET;
    tf2::Quaternion tube_q;
    tube_q.setRPY(0.0, M_PI / 2.0, 0.0);   // cylinder z -> TCP x
    aco.object.primitive_poses[0].orientation = tf2::toMsg(tube_q);
    aco.object.operation = aco.object.ADD;
    aco.touch_links = full_model->getJointModelGroup("gripper")->getLinkModelNames();
    psi.applyAttachedCollisionObject(aco);

    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    psm->requestPlanningSceneState("/get_planning_scene");
  }

  // Snapshot AFTER publishing, so the local copy and move_group agree.
  planning_scene::PlanningScenePtr bench_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO ro(psm);
    bench_scene = planning_scene::PlanningScene::clone(ro);
  }
  {
    std::vector<const moveit::core::AttachedBody *> attached;
    bench_scene->getCurrentState().getAttachedBodies(attached);
    std::cout << "scene: " << bench_scene->getWorld()->size() << " world objects, "
              << attached.size() << " attached\n";
    if (attached.empty())
      std::cout << "  WARNING: the carried tube did not attach; results will not "
                   "reflect a loaded gripper\n";
  }

  reduced.syncWorld(*bench_scene, { "carried_tube" });

  const moveit::core::RobotState reference = bench_scene->getCurrentState();
  const auto & kin = reduced.kinematics();

  // ---- build the query set ----------------------------------------------
  struct Query
  {
    std::string name;
    Eigen::Matrix<double, 7, 1> start;
    Eigen::Vector3d goal_tcp;
  };
  std::vector<Query> queries;

  // A configuration that puts the TCP at `tcp`, tool upright, collision free on
  // the FULL model. Returns false if no roll angle works.
  //
  // Both endpoints have to pass this or the query is not a fair test. A goal no
  // method can reach still costs the 7-DOF planner its entire time budget
  // before it gives up, while the reduced side rejects it in microseconds
  // during goal IK -- so leaving infeasible queries in would hand the reduced
  // method a large and completely fake win. (The first version of this
  // benchmark did exactly that: at z = 0.32 every tube station is past the
  // 0.2433 m reach, and all it measured was how fast each method fails.)
  const auto feasible = [&](const Eigen::Vector3d & tcp,
                            Eigen::Matrix<double, 7, 1> & out) {
    for (int i = 0; i < 24; ++i)
    {
      const double phi = 2.0 * M_PI * i / 24;
      const Eigen::Vector3d W = tcp - lifter.flangeOffset() * PathLifter::flangeDirection(phi);
      for (const auto & q4 : kin.positionIkSampled(W, 12))
      {
        const auto lifted = lifter.lift({ q4 }, phi, phi);
        if (!lifted.success) continue;
        std::size_t bad = 0;
        if (!lifter.validate(*bench_scene, reference, lifted.joints, bad)) continue;
        out = lifted.joints.front();
        return true;
      }
    }
    return false;
  };

  int dropped = 0;
  for (double z : heights)
    for (std::size_t a = 0; a < std::size(kStations); ++a)
      for (std::size_t b = 0; b < std::size(kStations); ++b)
      {
        if (a == b) continue;
        const Eigen::Vector3d tcp_a(kStations[a].x, kStations[a].y, z);
        const Eigen::Vector3d tcp_b(kStations[b].x, kStations[b].y, z);

        Eigen::Matrix<double, 7, 1> start_q, goal_q;
        if (!feasible(tcp_a, start_q) || !feasible(tcp_b, goal_q))
        {
          ++dropped;
          continue;
        }
        queries.push_back({ std::string(kStations[a].name) + "->" + kStations[b].name +
                                "@" + std::to_string(z).substr(0, 4),
                            start_q, tcp_b });
      }

  if (max_queries > 0 && queries.size() > static_cast<std::size_t>(max_queries))
  {
    // Take an even spread rather than a prefix, so the subset still covers
    // both short hops and long traverses across the workspace.
    std::vector<Query> subset;
    for (int i = 0; i < max_queries; ++i)
      subset.push_back(queries[i * queries.size() / max_queries]);
    queries.swap(subset);
  }

  std::cout << "\ndropped " << dropped
            << " station pairs as infeasible for BOTH methods (out of reach or in collision)\n";
  std::cout << "queries: " << queries.size() << "   repetitions: " << reps
            << "   budget: " << budget << " s   group: " << group << "\n";
  std::cout << "tcp link: " << tcp_link << "   frame: " << frame << "\n\n";

  Stats s_strict, s_free, s_reduced, s_control;
  std::ofstream csv_out;
  if (!csv.empty())
  {
    csv_out.open(csv);
    csv_out << "query,method,rep,success,validated,wall_s,reported_s,path_len,max_tilt\n";
  }

  const auto constraint = uprightConstraint(tcp_link, frame);

  // Flush every row. Without this the whole file sits in the stream buffer
  // until the process exits cleanly -- so an interrupted run, or one killed
  // because move_group died under it, leaves a zero-byte CSV and an hour of
  // work with nothing to show for it. That happened; hence the flush.
  const auto record = [&](const std::string & qname, const char * method, int rep,
                          const Sample & smp) {
    if (csv_out)
      csv_out << qname << "," << method << "," << rep << "," << smp.success << ","
              << smp.validated << "," << smp.wall << "," << smp.reported << ","
              << smp.path_length << "," << smp.max_tilt << "\n" << std::flush;
  };

  // Watchdog for a dead move_group.
  //
  // A genuine planning failure burns the whole time budget. An action server
  // that is gone fails instantly. So a run of instant failures means the
  // server died, not that the problem is hard -- and continuing would silently
  // record hundreds of meaningless "failures".
  int instant_failures = 0;
  bool aborted = false;
  const auto watchdog = [&](const Sample & smp) {
    if (!smp.success && smp.wall < 0.5)
      ++instant_failures;
    else
      instant_failures = 0;

    if (instant_failures >= 6 && !aborted)
    {
      aborted = true;
      std::cout << "\nABORTING: six consecutive planning calls failed in under "
                   "0.5 s each. move_group is almost certainly gone. Results so "
                   "far are in the CSV.\n";
    }
    return aborted;
  };

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t done = 0;
  const std::size_t total = queries.size() * static_cast<std::size_t>(reps);

  for (const auto & q : queries)
  {
    moveit::core::RobotState start_state(reference);
    for (int k = 0; k < 7; ++k)
    {
      const std::string n = "joint" + std::to_string(k + 1);
      start_state.setJointPositions(n, &q.start[k]);
    }
    start_state.update();

    for (int rep = 0; rep < reps; ++rep)
    {
      // ---- 7-DOF, single fixed goal pose ---------------------------------
      {
        Sample smp;
        mg.setStartState(start_state);
        mg.clearPoseTargets();
        mg.setPathConstraints(constraint);
        geometry_msgs::msg::Pose p;
        p.position.x = q.goal_tcp.x(); p.position.y = q.goal_tcp.y(); p.position.z = q.goal_tcp.z();
        p.orientation = uprightQuat(0.0);
        mg.setPoseTarget(p, tcp_link);

        MGI::Plan plan;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        smp.wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        smp.success = ok;
        if (ok)
        {
          smp.reported = MRC_PLAN_TIME(plan);
          smp.path_length = trajPathLength(MRC_PLAN_TRAJ(plan));
          smp.max_tilt = maxTilt(MRC_PLAN_TRAJ(plan), start_state, tcp_link);
        }
        mg.clearPathConstraints();
        s_strict.add(smp);
        record(q.name, "7dof-strict", rep, smp);
        watchdog(smp);
      }

      // ---- 7-DOF, no path constraint (control) ---------------------------
      //
      // Not a candidate method -- it is free to tip the tube over and spill
      // everything. It exists to separate "the orientation constraint is what
      // makes this hard" from "the benchmark is misconfigured". If this one
      // also fails, the problem is the setup, not the constraint.
      {
        Sample smp;
        mg.setStartState(start_state);
        mg.clearPoseTargets();
        mg.clearPathConstraints();
        std::vector<geometry_msgs::msg::Pose> targets;
        for (int i = 0; i < roll_fan; ++i)
        {
          geometry_msgs::msg::Pose p;
          p.position.x = q.goal_tcp.x(); p.position.y = q.goal_tcp.y(); p.position.z = q.goal_tcp.z();
          p.orientation = uprightQuat(2.0 * M_PI * i / roll_fan);
          targets.push_back(p);
        }
        mg.setPoseTargets(targets, tcp_link);

        MGI::Plan plan;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        smp.wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        smp.success = ok;
        if (ok)
        {
          smp.reported = MRC_PLAN_TIME(plan);
          smp.path_length = trajPathLength(MRC_PLAN_TRAJ(plan));
          smp.max_tilt = maxTilt(MRC_PLAN_TRAJ(plan), start_state, tcp_link);
        }
        s_control.add(smp);
        record(q.name, "7dof-nocon", rep, smp);
        watchdog(smp);
      }

      // ---- 7-DOF, fan of goal rolls --------------------------------------
      {
        Sample smp;
        mg.setStartState(start_state);
        mg.clearPoseTargets();
        mg.setPathConstraints(constraint);
        std::vector<geometry_msgs::msg::Pose> targets;
        for (int i = 0; i < roll_fan; ++i)
        {
          geometry_msgs::msg::Pose p;
          p.position.x = q.goal_tcp.x(); p.position.y = q.goal_tcp.y(); p.position.z = q.goal_tcp.z();
          p.orientation = uprightQuat(2.0 * M_PI * i / roll_fan);
          targets.push_back(p);
        }
        mg.setPoseTargets(targets, tcp_link);

        MGI::Plan plan;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        smp.wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        smp.success = ok;
        if (ok)
        {
          smp.reported = MRC_PLAN_TIME(plan);
          smp.path_length = trajPathLength(MRC_PLAN_TRAJ(plan));
          smp.max_tilt = maxTilt(MRC_PLAN_TRAJ(plan), start_state, tcp_link);
        }
        mg.clearPathConstraints();
        s_free.add(smp);
        record(q.name, "7dof-free", rep, smp);
        watchdog(smp);
      }

      // ---- reduced --------------------------------------------------------
      {
        Sample smp;
        const auto t0 = std::chrono::steady_clock::now();

        const auto res = reduced.planToToolPose(q.start.head<4>(), q.goal_tcp,
                                                lifter.flangeOffset(), roll_fan);
        if (res.success)
        {
          const double roll_start = lifter.rollOf(start_state);
          const double roll_end =
              PathLifter::rollFor(kin.wristCentre(res.path.back()), q.goal_tcp);
          const auto lifted = lifter.lift(res.path, roll_start, roll_end);
          if (lifted.success)
          {
            std::size_t bad = 0;
            smp.validated = lifter.validate(*bench_scene, reference, lifted.joints, bad);
            smp.success = true;
            smp.path_length = jointPathLength(lifted.joints);
            smp.max_tilt = lifted.max_tilt;
          }
        }
        smp.wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        smp.reported = res.planning_time;
        s_reduced.add(smp);
        record(q.name, "reduced", rep, smp);
      }

      ++done;
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
      const double eta = done ? elapsed / done * (total - done) : 0.0;
      std::cout << "[" << done << "/" << total << "] " << std::left << std::setw(22)
                << q.name << std::right << "  strict " << (s_strict.ok ? "" : "")
                << std::setw(3) << s_strict.ok << "/" << std::setw(3) << s_strict.runs
                << "   free " << std::setw(3) << s_free.ok << "/" << std::setw(3) << s_free.runs
                << "   reduced " << std::setw(3) << s_reduced.ok << "/" << std::setw(3)
                << s_reduced.runs << "   elapsed " << std::setprecision(0) << elapsed
                << "s  eta " << eta << "s\n" << std::setprecision(6) << std::flush;
    }
    if (aborted) break;
  }

  // ---- report ------------------------------------------------------------
  const auto row = [](const char * name, const Stats & st) {
    std::cout << std::left << std::setw(14) << name << std::right << std::fixed
              << std::setw(8) << std::setprecision(1)
              << (st.runs ? 100.0 * st.ok / st.runs : 0.0) << " %"
              << std::setw(11) << std::setprecision(4) << median(st.wall)
              << std::setw(11) << median(st.reported)
              << std::setw(11) << std::setprecision(3) << mean(st.len)
              << std::setw(10) << std::setprecision(3) << (st.tilt.empty() ? 0.0 : *std::max_element(st.tilt.begin(), st.tilt.end()))
              << std::setw(10) << std::setprecision(1)
              << (st.ok ? 100.0 * st.validated / st.ok : 0.0) << " %\n";
  };

  std::cout << "\n" << std::left << std::setw(14) << "method" << std::right
            << std::setw(10) << "success" << std::setw(11) << "wall med"
            << std::setw(11) << "plan med" << std::setw(11) << "path len"
            << std::setw(10) << "max tilt" << std::setw(12) << "validated" << "\n";
  std::cout << std::string(79, '-') << "\n";
  row("7dof-strict", s_strict);
  row("7dof-free", s_free);
  row("reduced", s_reduced);
  std::cout << std::string(79, '-') << "\n";
  row("7dof-nocon*", s_control);
  std::cout << "* control only: no orientation constraint, so it may tip the tube\n";
  std::cout << "\ntilt tolerance was " << TILT_TOLERANCE
            << " rad; any max tilt above that is a constraint violation\n";
  if (!csv.empty()) std::cout << "per-run data written to " << csv << "\n";

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
