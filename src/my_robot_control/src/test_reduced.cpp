// Offline validation for the reduced-planning pipeline (stages 3 and 4).
//
// Runs without ROS, without a robot and without move_group: it loads the
// generated reduced model, rebuilds the same collision scene simple_move.cpp
// publishes, and exercises the kinematics and the planner directly.
//
//   ./test_reduced [--urdf ...] [--srdf ...] [--group arm_positioning]
//
// Checks, in order:
//   1  analytic FK against the robot model            (the constructor's own test)
//   2  position IK round-trip over random states
//   3  wrist ZYZ IK round-trip, on the full 7-DOF model
//   4  blob alignment: does the blob really stay world-vertical
//   5  a transit plan through the real scene, timed

#include "my_robot_control/reduced_planner.hpp"
#include "my_robot_control/srs_kinematics.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

using namespace my_robot_control;

namespace
{

// Scene constants, mirrored from simple_move.cpp.
constexpr double TABLE_SIZE_XY = 1.5, TABLE_THICKNESS = 0.04, TABLE_CENTER_Z = -0.03;
constexpr double TUBE_HEIGHT = 0.13, TUBE_RADIUS = 0.012, TUBE_CENTER_Z = 0.08;
constexpr double MIXER_HEIGHT = 0.17, MIXER_RADIUS = 0.06, MIXER_CENTER_Z = 0.08;
constexpr int NUM_TUBES = 5;
const double TUBE_OBJ_X[NUM_TUBES] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
constexpr double TUBE_OBJ_Y = -0.345, TUBE_Y_OFFSET = 0.02;
constexpr double MIXER_OBJ_X = -0.32, MIXER_OBJ_Y = -0.33, MIXER_Y_OFFSET = 0.10;

std::string readFile(const std::string & p)
{
  std::ifstream in(p);
  if (!in) throw std::runtime_error("cannot open " + p);
  std::ostringstream ss; ss << in.rdbuf(); return ss.str();
}

moveit::core::RobotModelPtr loadModel(const std::string & urdf_path, const std::string & srdf_path)
{
  const auto u = urdf::parseURDF(readFile(urdf_path));
  if (!u) throw std::runtime_error("bad URDF " + urdf_path);
  auto s = std::make_shared<srdf::Model>();
  if (!s->initString(*u, readFile(srdf_path))) throw std::runtime_error("bad SRDF " + srdf_path);
  return std::make_shared<moveit::core::RobotModel>(u, s);
}

void addBox(planning_scene::PlanningScene & sc, const std::string & id,
            double sx, double sy, double sz, double x, double y, double z)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  sc.getWorldNonConst()->addToObject(
      id, shapes::ShapeConstPtr(new shapes::Box(sx, sy, sz)), p);
}

void addCylinder(planning_scene::PlanningScene & sc, const std::string & id,
                 double r, double h, double x, double y, double z)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  sc.getWorldNonConst()->addToObject(
      id, shapes::ShapeConstPtr(new shapes::Cylinder(r, h)), p);
}

void buildScene(planning_scene::PlanningScene & sc)
{
  addBox(sc, "table_base", TABLE_SIZE_XY, TABLE_SIZE_XY, TABLE_THICKNESS, 0, 0, TABLE_CENTER_Z);
  addBox(sc, "obstacle_box", 0.7, 0.2, 0.05, 0.0, 0.18, 0.02);
  addBox(sc, "obstacle_box_2", 0.4, 0.2, 0.3, -0.6, 0.1, 0.03);
  for (int i = 0; i < NUM_TUBES; ++i)
    addCylinder(sc, "tube_" + std::to_string(i + 1), TUBE_RADIUS, TUBE_HEIGHT,
                TUBE_OBJ_X[i], TUBE_OBJ_Y, TUBE_CENTER_Z);
  addCylinder(sc, "mixer", MIXER_RADIUS, MIXER_HEIGHT,
              MIXER_OBJ_X, MIXER_OBJ_Y + MIXER_Y_OFFSET, MIXER_CENTER_Z);
}

std::string arg(int argc, char ** argv, const char * flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i)
    if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
  return fallback;
}

int failures = 0;
void check(bool ok, const std::string & what, const std::string & detail = "")
{
  std::cout << (ok ? "  [ ok ] " : "  [FAIL] ") << what;
  if (!detail.empty()) std::cout << "   " << detail;
  std::cout << "\n";
  if (!ok) ++failures;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string gen = arg(argc, argv, "--gen", "generated");
  const std::string red_urdf = arg(argc, argv, "--urdf", gen + "/reduced_arm.urdf");
  const std::string red_srdf = arg(argc, argv, "--srdf", gen + "/reduced_arm.srdf");
  const std::string full_urdf = arg(argc, argv, "--full-urdf", "");
  const std::string full_srdf = arg(argc, argv, "--full-srdf", "");
  const std::string group = arg(argc, argv, "--group", "arm_positioning");

  std::cout << std::fixed << std::setprecision(6);

  // ---- 1: model + analytic FK -------------------------------------------
  std::cout << "\n=== 1. reduced model and analytic FK ===\n";
  ReducedPlanner planner;
  ReducedPlanner::Options opt;
  opt.urdf_path = red_urdf;
  opt.srdf_path = red_srdf;
  opt.group = group;
  opt.planning_time = 5.0;
  opt.rng_seed = 42;
  planner.initialize(opt);

  const SrsKinematics & kin = planner.kinematics();
  std::cout << "  group        : " << group << "\n";
  std::cout << "  shoulder     : " << kin.shoulder().transpose() << "\n";
  std::cout << "  upper arm    : " << kin.upperArm() << " m\n";
  std::cout << "  forearm      : " << kin.forearm() << " m\n";
  std::cout << "  reach        : " << kin.minReach() << " .. " << kin.maxReach() << " m\n";
  check(kin.selfTestError() < 1e-4, "analytic FK matches the robot model",
        "max error " + std::to_string(kin.selfTestError()) + " m");

  // ---- 2: position IK round-trip ----------------------------------------
  std::cout << "\n=== 2. position IK round-trip ===\n";
  {
    std::mt19937 rng(7);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<>(a, b)(rng); };
    int tried = 0, solved = 0;
    double worst = 0.0;
    for (int i = 0; i < 5000; ++i)
    {
      Eigen::Vector4d q;
      for (int k = 0; k < 4; ++k) q[k] = U(kin.lower()[k], kin.upper()[k]);
      const Eigen::Vector3d W = kin.wristCentre(q);
      ++tried;

      // Hold the arm angle at the value that generated the sample: with q3
      // fixed the problem is square, so a correct solver must recover it.
      const auto sols = kin.positionIk(W, q[2]);
      if (sols.empty()) continue;
      ++solved;

      double best = 1e9;
      for (const auto & s : sols) best = std::min(best, (kin.wristCentre(s) - W).norm());
      worst = std::max(worst, best);
    }
    check(solved == tried, "every reachable wrist centre has an IK solution",
          std::to_string(solved) + "/" + std::to_string(tried));
    check(worst < 1e-6, "IK solutions reproduce the target wrist centre",
          "max residual " + std::to_string(worst) + " m");
  }

  // ---- 3: wrist ZYZ IK ---------------------------------------------------
  std::cout << "\n=== 3. wrist ZYZ IK (needs the full 7-DOF model) ===\n";
  if (full_urdf.empty() || full_srdf.empty())
  {
    std::cout << "  [skip] pass --full-urdf and --full-srdf to run this\n";
  }
  else
  {
    const auto full = loadModel(full_urdf, full_srdf);
    SrsKinematics fkin(full, "link5");   // link5's origin IS the wrist centre
    moveit::core::RobotState st(full);
    st.setToDefaultValues();

    std::mt19937 rng(11);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<>(a, b)(rng); };
    int tried = 0, solved = 0;
    double worst = 0.0;
    for (int i = 0; i < 3000; ++i)
    {
      Eigen::Matrix<double, 7, 1> q;
      for (int k = 0; k < 7; ++k) q[k] = U(fkin.lower()[k], fkin.upper()[k]);
      for (int k = 0; k < 7; ++k)
      {
        const std::string n = "joint" + std::to_string(k + 1);
        st.setJointPositions(n, &q[k]);
      }
      st.update();
      const Eigen::Matrix3d R_tool = st.getGlobalLinkTransform("link7").rotation();

      ++tried;
      const auto sols = fkin.wristIk(q.head<4>(), R_tool);
      if (sols.empty()) continue;
      ++solved;

      double best = 1e9;
      for (const auto & s : sols)
      {
        for (int k = 0; k < 3; ++k)
        {
          const std::string n = "joint" + std::to_string(k + 5);
          const double v = s[k];
          st.setJointPositions(n, &v);
        }
        st.update();
        const Eigen::Matrix3d got = st.getGlobalLinkTransform("link7").rotation();
        best = std::min(best, (got - R_tool).cwiseAbs().maxCoeff());
      }
      worst = std::max(worst, best);
    }
    check(solved > tried * 0.9, "wrist IK finds a branch for most orientations",
          std::to_string(solved) + "/" + std::to_string(tried));
    // Same 1.5708-vs-pi/2 rounding as the FK self-test, propagated through
    // three joints instead of one, so the residual is ~1e-5 rather than ~1e-6.
    check(worst < 1e-4, "wrist IK reproduces the tool orientation",
          "max residual " + std::to_string(worst));
  }

  // ---- 4: blob alignment -------------------------------------------------
  std::cout << "\n=== 4. blob stays world-vertical ===\n";
  {
    moveit::core::RobotState st(planner.model());
    st.setToDefaultValues();
    std::mt19937 rng(3);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<>(a, b)(rng); };
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
      for (int k = 0; k < 4; ++k)
      {
        const std::string n = "joint" + std::to_string(k + 1);
        const double v = U(kin.lower()[k], kin.upper()[k]);
        st.setJointPositions(n, &v);
      }
      st.update();
      planner.alignBlob(st);
      st.update();
      const Eigen::Vector3d z = st.getGlobalLinkTransform("wrist_blob").rotation().col(2);
      // Measure the horizontal component, not acos(z.z()). Near perfect
      // alignment z.z() is 1 - O(1e-16) and acos amplifies that to ~1e-8, which
      // says nothing about the alignment and everything about acos.
      worst = std::max(worst, std::hypot(z.x(), z.y()));
    }
    check(worst < 1e-9, "blob z axis stays along world z for any arm pose",
          "max tilt " + std::to_string(worst));
  }

  // ---- 5: a real transit plan -------------------------------------------
  std::cout << "\n=== 5. transit plan through the task scene ===\n";
  {
    planning_scene::PlanningScene live(planner.model());
    buildScene(live);
    planner.syncWorld(live, { "tube_3" });   // tube_3 is the one being carried

    std::cout << "  world objects: ";
    for (const auto & id : planner.scene()->getWorld()->getObjectIds()) std::cout << id << " ";
    std::cout << "\n";

    const bool three_dof = (group == "arm_positioning_3dof");
    const Eigen::Vector4d start(0.0, 0.0, 0.0, -0.7395);
    std::cout << "  start wrist centre : " << kin.wristCentre(start).transpose()
              << (planner.isValid(start) ? "   (valid)" : "   (IN COLLISION)") << "\n";

    // How much of the workspace survives the blob, height by height. This is
    // the real measure of how much the conservative surrogate costs: IK-
    // reachable tells us what the ARM can do, blob-free tells us what the
    // REDUCED PLANNER will admit, and the gap between them is the price.
    std::cout << "\n  wrist-centre coverage at each transit height\n";
    std::cout << "     z (m)   in reach   blob-free   admitted\n";
    double best_z = 0.0;
    int best_free = -1;
    for (double z = 0.18; z <= 0.361; z += 0.02)
    {
      int reach = 0, free = 0;
      for (double x = -0.24; x <= 0.241; x += 0.02)
        for (double y = -0.24; y <= 0.241; y += 0.02)
        {
          const Eigen::Vector3d W(x, y, z);
          // The 3-DOF group cannot move joint3, so sweeping the arm angle here
          // would report coverage the planner has no way to reach.
          const auto sols = three_dof ? kin.positionIk(W, 0.0)
                                      : kin.positionIkSampled(W, 12);
          if (sols.empty()) continue;
          ++reach;
          for (const auto & q : sols)
            if (planner.isValid(q)) { ++free; break; }
        }
      const double pct = reach ? 100.0 * free / reach : 0.0;
      std::cout << "     " << std::setprecision(2) << z << "     " << std::setw(6) << reach
                << "     " << std::setw(7) << free << "     " << std::setprecision(1)
                << std::setw(6) << pct << " %\n";
      if (free > best_free) { best_free = free; best_z = z; }
    }
    std::cout << std::setprecision(6);

    // Plan across the workspace at the height that survives best, between two
    // points on opposite sides of the tube rack.
    const Eigen::Vector3d goal(-0.14, 0.05, best_z);
    std::cout << "\n  planning to " << goal.transpose() << " at the best height ("
              << std::setprecision(2) << best_z << " m)\n"
              << std::setprecision(6);
    const auto res = planner.planToWristCentre(start, goal);

    std::cout << "  goal states   : " << res.goal_states << "\n";
    std::cout << "  states checked: " << res.states_checked << "\n";
    std::cout << "  planning time : " << res.planning_time << " s\n";
    check(res.success, "reduced planner found a transit path", res.message);

    if (res.success)
    {
      bool all_valid = true;
      for (const auto & q : res.path) all_valid = all_valid && planner.isValid(q);
      const double err = (kin.wristCentre(res.path.back()) - goal).norm();
      std::cout << "  waypoints     : " << res.path.size() << "\n";
      check(all_valid, "every waypoint is collision free in the reduced model");
      check(err < 1e-6, "path ends at the requested wrist centre",
            "error " + std::to_string(err) + " m");
    }
  }

  std::cout << "\n" << (failures == 0 ? "ALL CHECKS PASSED" : std::to_string(failures) + " CHECK(S) FAILED")
            << "\n\n";
  return failures == 0 ? 0 : 1;
}
