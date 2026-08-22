// Does the 5-DOF exact planner handle the transits the swept-volume planner
// could not?
//
// Runs the SAME station-to-station queries the benchmark used, where the blob
// approach scored 0/30 and the current 7-DOF pipeline scored 60%. Offline: no
// move_group, no robot.
//
//   ./test_manifold --urdf ... --srdf ... [--transit-z 0.24] [--lock-arm-angle]

#include "my_robot_control/manifold_planner.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace my_robot_control;

namespace
{

constexpr double TUBE_HEIGHT = 0.13, TUBE_RADIUS = 0.012;
constexpr double GRIPPER_CLOSED = -1.2;

struct Station { const char * name; double x, y; };
const Station kStations[] = {
  { "tube1", -0.17, -0.22 }, { "tube2", -0.10, -0.22 }, { "tube3", 0.00, -0.22 },
  { "tube4", 0.10, -0.22 },  { "tube5", 0.17, -0.22 },  { "mixer", -0.20, -0.11 },
};

std::string readFile(const std::string & p)
{
  std::ifstream in(p);
  if (!in) throw std::runtime_error("cannot open " + p);
  std::ostringstream ss; ss << in.rdbuf(); return ss.str();
}

std::string arg(int argc, char ** argv, const char * flag, const std::string & fb)
{
  for (int i = 1; i + 1 < argc; ++i)
    if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
  return fb;
}
bool has(int argc, char ** argv, const char * flag)
{
  for (int i = 1; i < argc; ++i) if (std::strcmp(argv[i], flag) == 0) return true;
  return false;
}

void addBox(planning_scene::PlanningScene & sc, const std::string & id,
            double sx, double sy, double sz, double x, double y, double z)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  sc.getWorldNonConst()->addToObject(id, shapes::ShapeConstPtr(new shapes::Box(sx, sy, sz)), p);
}
void addCyl(planning_scene::PlanningScene & sc, const std::string & id,
            double r, double h, double x, double y, double z)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  sc.getWorldNonConst()->addToObject(id, shapes::ShapeConstPtr(new shapes::Cylinder(r, h)), p);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string urdf_path = arg(argc, argv, "--urdf", "");
  const std::string srdf_path = arg(argc, argv, "--srdf", "");
  const double z = std::stod(arg(argc, argv, "--transit-z", "0.24"));
  const bool lock = has(argc, argv, "--lock-arm-angle");
  const double budget = std::stod(arg(argc, argv, "--budget", "2.0"));

  if (urdf_path.empty() || srdf_path.empty())
  {
    std::cerr << "need --urdf and --srdf\n";
    return 2;
  }

  const auto u = urdf::parseURDF(readFile(urdf_path));
  auto s = std::make_shared<srdf::Model>();
  s->initString(*u, readFile(srdf_path));
  const auto model = std::make_shared<moveit::core::RobotModel>(u, s);

  // Same scene as the benchmark.
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  addBox(*scene, "table_base", 1.5, 1.5, 0.04, 0, 0, -0.03);
  addBox(*scene, "obstacle_box", 0.7, 0.2, 0.05, 0.0, 0.18, 0.02);
  addBox(*scene, "obstacle_box_2", 0.4, 0.2, 0.3, -0.6, 0.1, 0.03);
  const double tube_x[5] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
  for (int i = 0; i < 5; ++i)
    addCyl(*scene, "tube_" + std::to_string(i + 1), TUBE_RADIUS, TUBE_HEIGHT,
           tube_x[i], -0.345, 0.08);
  addCyl(*scene, "mixer", 0.06, 0.17, -0.32, -0.23, 0.08);

  // Reference state: gripper closed on the carried tube, tube attached.
  moveit::core::RobotState ref(model);
  ref.setToDefaultValues();
  {
    const double g[6] = { GRIPPER_CLOSED, -GRIPPER_CLOSED, GRIPPER_CLOSED,
                          -GRIPPER_CLOSED, -GRIPPER_CLOSED, -GRIPPER_CLOSED };
    const char * gn[6] = { "endeffector_gripper", "gripperbase_to_armgearleft",
                           "gripperbase_to_armsimpleright", "gripperbase_to_armsimpleleft",
                           "armgearright_to_fingerright", "armgearleft_to_fingerleft" };
    for (int i = 0; i < 6; ++i) ref.setJointPositions(gn[i], &g[i]);
    ref.update();
  }

  ManifoldPlanner planner;
  ManifoldPlanner::Options opt;
  opt.model = model;
  opt.lock_arm_angle = lock;
  opt.planning_time = budget;
  opt.rng_seed = 42;
  planner.initialize(opt);

  // Attach the carried tube to the reference state, in the TCP frame.
  {
    shapes::ShapeConstPtr tube(new shapes::Cylinder(TUBE_RADIUS, TUBE_HEIGHT));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(-0.030, 0.0, 0.135);
    pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const auto * eef = model->getJointModelGroup("gripper");
    std::vector<std::string> touch = eef ? eef->getLinkModelNames() : std::vector<std::string>{};
    ref.attachBody("carried_tube", Eigen::Isometry3d::Identity(), { tube }, { pose },
                   touch, "link7");
    ref.update();
  }
  planner.setScene(scene, ref);

  const double off = planner.flangeOffset();
  const auto & kin = planner.kinematics();

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "\n5-DOF exact manifold planner"
            << (lock ? "  (arm angle locked -> 4-D)" : "  (5-D)") << "\n"
            << "transit z = " << z << " m, budget " << budget << " s\n"
            << "flange offset " << off << " m\n\n";

  // Start configurations at each station, same construction as the benchmark.
  const auto configAt = [&](const Eigen::Vector3d & tcp,
                            ManifoldPlanner::JointVector & out) {
    for (int i = 0; i < 24; ++i)
    {
      const double phi = -M_PI + 2.0 * M_PI * i / 24;
      const Eigen::Vector3d W = tcp - off * PathLifter::flangeDirection(phi);
      for (const auto & a : kin.positionIkSampled(W, 12))
      {
        ManifoldPlanner::JointVector q;
        for (int br : { 0, 1 })
          if (planner.stateToConfig(a, phi, br, q) && planner.isValid(q)) { out = q; return true; }
      }
    }
    return false;
  };

  // ---- diagnostic: does the start state survive its own round trip? -------
  {
    ManifoldPlanner::JointVector start;
    const Eigen::Vector3d A(kStations[0].x, kStations[0].y, z);
    if (configAt(A, start))
    {
      moveit::core::RobotState st(ref);
      const char * jn[7] = { "joint1", "joint2", "joint3", "joint4",
                             "joint5", "joint6", "joint7" };
      for (int k = 0; k < 7; ++k) st.setJointPositions(jn[k], &start[k]);
      st.update();
      const Eigen::Vector3d W = kin.wristCentre(start.head<4>());
      const Eigen::Vector3d tcp = st.getGlobalLinkTransform("link7").translation();
      const double phi = PathLifter::rollFor(W, tcp);

      std::cout << "  [diag] start valid          : " << planner.isValid(start) << "\n";
      std::cout << "  [diag] tcp target           : " << A.transpose() << "\n";
      std::cout << "  [diag] tcp actual           : " << tcp.transpose() << "\n";
      std::cout << "  [diag] recovered phi        : " << phi << "\n";
      for (int br : { 0, 1 })
      {
        ManifoldPlanner::JointVector q;
        if (!planner.stateToConfig(start.head<4>(), phi, br, q))
        {
          std::cout << "  [diag] branch " << br << ": no wrist solution\n";
          continue;
        }
        std::cout << "  [diag] branch " << br << ": wrist err "
                  << (q.tail<3>() - start.tail<3>()).cwiseAbs().sum()
                  << "   valid " << planner.isValid(q) << "\n";
      }
      std::cout << "\n";
    }
  }

  int tried = 0, ok = 0;
  double t_sum = 0.0, worst_tilt = 0.0;
  std::size_t checks_sum = 0;

  for (std::size_t a = 0; a < std::size(kStations); ++a)
    for (std::size_t b = 0; b < std::size(kStations); ++b)
    {
      if (a == b) continue;
      const Eigen::Vector3d A(kStations[a].x, kStations[a].y, z);
      const Eigen::Vector3d B(kStations[b].x, kStations[b].y, z);

      ManifoldPlanner::JointVector start;
      if (!configAt(A, start)) continue;

      ++tried;
      const auto r = planner.planToToolPose(start, B);
      t_sum += r.planning_time;
      checks_sum += r.states_checked;
      if (r.success)
      {
        ++ok;
        worst_tilt = std::max(worst_tilt, r.max_tilt);
      }

      std::cout << "  " << std::left << std::setw(6) << kStations[a].name << " -> "
                << std::setw(6) << kStations[b].name << std::right
                << (r.success ? "  OK  " : "  --  ")
                << std::setw(8) << r.planning_time * 1e3 << " ms"
                << std::setw(6) << r.path.size() << " wp"
                << std::setw(6) << r.goal_states << " goals"
                << "  tilt " << r.max_tilt
                << (r.success ? "" : ("   " + r.message)) << "\n";
    }

  std::cout << "\n----------------------------------------------------------\n";
  std::cout << "  transits attempted : " << tried << "\n";
  std::cout << "  succeeded          : " << ok << "   ("
            << (tried ? 100.0 * ok / tried : 0.0) << " %)\n";
  std::cout << "  mean plan time     : " << (tried ? t_sum / tried * 1e3 : 0.0) << " ms\n";
  std::cout << "  mean states checked: " << (tried ? checks_sum / tried : 0) << "\n";
  std::cout << "  worst tool tilt    : " << worst_tilt << " rad\n";
  std::cout << "\n  for reference, on the same transits at this height:\n"
            << "    swept-volume (4-DOF) : 0 %\n"
            << "    7-DOF constrained    : 60 %, ~10 s, tilt up to 0.418 rad\n\n";
  return 0;
}
