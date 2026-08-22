// RViz preview of the reduced-planning pipeline.
//
// Plans a transit move with the reduced planner, lifts it back to 7 DOF in
// closed form, and animates the result on the PREVIEW model -- the whole real
// arm with the swept-volume blob attached at the wrist centre.
//
// The point of showing both together: the blob is supposed to contain every
// pose the wrist assembly can take under the upright constraint. On screen you
// can watch the real gripper and tube move inside the ring and never leave it.
// If they ever poke out, the sweep in wrist_blob_gen.py is wrong.
//
//   ros2 launch my_robot_control preview_reduced.launch.py
//
// No move_group and no robot. This node publishes /joint_states directly, and
// the obstacles as markers.

#include "my_robot_control/path_lifter.hpp"
#include "my_robot_control/reduced_planner.hpp"

#include <geometric_shapes/shapes.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <sstream>

using namespace my_robot_control;

namespace
{

constexpr double TUBE_HEIGHT = 0.13, TUBE_RADIUS = 0.012;
constexpr double GRIPPER_CLOSED = -1.2;

std::string readFile(const std::string & p)
{
  std::ifstream in(p);
  if (!in) throw std::runtime_error("cannot open " + p);
  std::ostringstream ss; ss << in.rdbuf(); return ss.str();
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "preview_reduced",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const std::string gen = node->get_parameter_or<std::string>("gen_dir", "generated");
  const std::string group = node->get_parameter_or<std::string>("group", "arm_positioning");
  const double z = node->get_parameter_or<double>("transit_z", 0.24);
  const double rate_hz = node->get_parameter_or<double>("rate", 30.0);
  const double dwell = node->get_parameter_or<double>("dwell", 1.0);

  auto js_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto mk_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/preview_obstacles", rclcpp::QoS(1).transient_local());

  // ---- planner ----------------------------------------------------------
  ReducedPlanner planner;
  ReducedPlanner::Options opt;
  opt.urdf_path = gen + "/reduced_arm.urdf";
  opt.srdf_path = gen + "/reduced_arm.srdf";
  opt.group = group;
  opt.planning_time = 5.0;
  planner.initialize(opt);

  // The lifter needs the real 7-DOF model. The preview URDF is exactly that,
  // plus the blob, so it serves for both.
  const auto preview_urdf = urdf::parseURDF(readFile(gen + "/preview_arm.urdf"));
  auto empty_srdf = std::make_shared<srdf::Model>();
  // srdfdom rejects a semantic description whose robot name differs from the
  // URDF's, so take the name from the URDF rather than inventing one.
  empty_srdf->initString(*preview_urdf, "<robot name='" + preview_urdf->getName() + "'/>");
  const auto preview_model =
      std::make_shared<moveit::core::RobotModel>(preview_urdf, empty_srdf);
  PathLifter lifter(preview_model, "link7");

  planning_scene::PlanningScene live(planner.model());
  addBox(live, "table_base", 1.5, 1.5, 0.04, 0, 0, -0.03);
  addBox(live, "obstacle_box", 0.7, 0.2, 0.05, 0.0, 0.18, 0.02);
  addBox(live, "obstacle_box_2", 0.4, 0.2, 0.3, -0.6, 0.1, 0.03);
  const double tube_x[5] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
  for (int i = 0; i < 5; ++i)
    addCyl(live, "tube_" + std::to_string(i + 1), TUBE_RADIUS, TUBE_HEIGHT, tube_x[i], -0.345, 0.08);
  addCyl(live, "mixer", 0.06, 0.17, -0.32, -0.23, 0.08);
  planner.syncWorld(live);

  // ---- markers for the obstacles ----------------------------------------
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    const auto & world = *live.getWorld();
    for (const auto & name : world.getObjectIds())
    {
      const auto obj = world.getObject(name);
      for (std::size_t k = 0; k < obj->shapes_.size(); ++k)
      {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.ns = name; m.id = id++;
        m.action = m.ADD;
        const auto & pose = obj->shape_poses_[k];
        m.pose.position.x = pose.translation().x();
        m.pose.position.y = pose.translation().y();
        m.pose.position.z = pose.translation().z();
        const Eigen::Quaterniond q(pose.rotation());
        m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
        m.color.a = 0.75; m.color.r = 0.55; m.color.g = 0.55; m.color.b = 0.6;

        const auto * s = obj->shapes_[k].get();
        if (const auto * b = dynamic_cast<const shapes::Box *>(s))
        {
          m.type = m.CUBE;
          m.scale.x = b->size[0]; m.scale.y = b->size[1]; m.scale.z = b->size[2];
        }
        else if (const auto * c = dynamic_cast<const shapes::Cylinder *>(s))
        {
          m.type = m.CYLINDER;
          m.scale.x = m.scale.y = 2.0 * c->radius; m.scale.z = c->length;
          m.color.r = 0.2; m.color.g = 0.6; m.color.b = 0.9;
        }
        else
        {
          continue;
        }
        arr.markers.push_back(m);
      }
    }

    // The carried tube. Published in the link7 frame, so TF carries it along
    // with the gripper and no per-frame update is needed.
    //
    // Without this the ring looks absurdly oversized: the tube sticks out
    // 201 mm from the wrist centre, and it -- not the hand -- is what sets the
    // radius. Seeing it swing around inside the ring is the whole point.
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "link7";
      m.ns = "carried_tube"; m.id = id++;
      m.action = m.ADD;
      m.type = m.CYLINDER;
      m.scale.x = m.scale.y = 2.0 * TUBE_RADIUS;
      m.scale.z = TUBE_HEIGHT;
      // In the TCP frame: 0.135 along +z, -0.030 along x, long axis on TCP x.
      m.pose.position.x = -0.030;
      m.pose.position.z = 0.135;
      const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
      m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
      m.color.a = 0.95; m.color.r = 0.95; m.color.g = 0.75; m.color.b = 0.1;
      arr.markers.push_back(m);
    }

    mk_pub->publish(arr);
    RCLCPP_INFO(node->get_logger(), "published %zu obstacle markers", arr.markers.size());
  }

  // ---- plan a loop of transit moves --------------------------------------
  const auto & kin = planner.kinematics();
  const double off = lifter.flangeOffset();

  // Pick the tour points from what is ACTUALLY admissible at this height,
  // rather than hardcoding the task's stations.
  //
  // The stations sit directly above the tube rack, and at normal transit
  // heights a 237 mm ring centred there swallows the neighbouring tubes -- so
  // the reduced planner rejects them, correctly. Hardcoding them would just
  // produce a preview that refuses to start. Sweep the workspace instead, keep
  // the valid wrist centres, and greedily choose a few far apart so the legs
  // are long enough to be worth watching.
  const auto first_config = [&](const Eigen::Vector3d & W, Eigen::Vector4d & out) {
    for (const auto & c : kin.positionIkSampled(W, 12))
      if (planner.isValid(c)) { out = c; return true; }
    return false;
  };

  std::vector<Eigen::Vector3d> valid;
  for (double x = -0.24; x <= 0.241; x += 0.02)
    for (double y = -0.24; y <= 0.241; y += 0.02)
    {
      Eigen::Vector4d c;
      const Eigen::Vector3d W(x, y, z);
      if (first_config(W, c)) valid.push_back(W);
    }

  if (valid.size() < 2)
  {
    RCLCPP_FATAL(node->get_logger(),
                 "only %zu admissible wrist centres at z=%.2f m. The blob needs "
                 "roughly z > 0.20 to clear the table; try transit_z:=0.30.",
                 valid.size(), z);
    rclcpp::shutdown();
    return 1;
  }

  std::vector<Eigen::Vector3d> stations{ valid.front() };
  while (stations.size() < 4)
  {
    const Eigen::Vector3d * best = nullptr;
    double best_d = -1.0;
    for (const auto & cand : valid)
    {
      double nearest = 1e9;
      for (const auto & s : stations) nearest = std::min(nearest, (cand - s).norm());
      if (nearest > best_d) { best_d = nearest; best = &cand; }
    }
    if (!best || best_d < 1e-6) break;
    stations.push_back(*best);
  }

  RCLCPP_INFO(node->get_logger(),
              "%zu admissible wrist centres at z=%.2f m; touring %zu of them",
              valid.size(), z, stations.size());

  Eigen::Vector4d q_now = Eigen::Vector4d::Zero();
  if (!first_config(stations.front(), q_now))
  {
    RCLCPP_FATAL(node->get_logger(), "lost the start configuration");
    rclcpp::shutdown();
    return 1;
  }

  sensor_msgs::msg::JointState js;
  js.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
              "blob_align_x", "blob_align_y",
              "endeffector_gripper", "gripperbase_to_armgearleft",
              "gripperbase_to_armsimpleright", "gripperbase_to_armsimpleleft",
              "armgearright_to_fingerright", "armgearleft_to_fingerleft" };

  moveit::core::RobotState align_state(planner.model());
  align_state.setToDefaultValues();

  const auto publish = [&](const Eigen::Matrix<double, 7, 1> & q7) {
    // The align joints depend only on q1..q4, so they can be read off the
    // reduced model even though we are drawing the full one.
    for (int k = 0; k < 4; ++k)
    {
      const std::string n = "joint" + std::to_string(k + 1);
      align_state.setJointPositions(n, &q7[k]);
    }
    align_state.update();
    planner.alignBlob(align_state);
    align_state.update();

    js.header.stamp = node->now();
    js.position = { q7[0], q7[1], q7[2], q7[3], q7[4], q7[5], q7[6],
                    *align_state.getJointPositions("blob_align_x"),
                    *align_state.getJointPositions("blob_align_y"),
                    GRIPPER_CLOSED, -GRIPPER_CLOSED, GRIPPER_CLOSED,
                    -GRIPPER_CLOSED, -GRIPPER_CLOSED, -GRIPPER_CLOSED };
    js_pub->publish(js);
  };

  rclcpp::Rate rate(rate_hz);
  std::size_t target = 1;

  RCLCPP_INFO(node->get_logger(),
              "previewing at transit z=%.2f m, group '%s'. Ctrl-C to stop.", z, group.c_str());

  while (rclcpp::ok())
  {
    const Eigen::Vector3d goal = stations[target % stations.size()];
    const auto res = planner.planToToolPose(q_now, goal, off, 24);

    if (!res.success)
    {
      RCLCPP_WARN(node->get_logger(), "station %zu unreachable: %s",
                  target % stations.size(), res.message.c_str());
      target++;
      continue;
    }

    // Roll at the start of the leg is whatever the previous leg ended on;
    // seed the very first leg with 0.
    static double roll_a = 0.0;
    const double roll_b = PathLifter::rollFor(kin.wristCentre(res.path.back()), goal);

    const auto lifted = lifter.lift(res.path, roll_a, roll_b);
    if (!lifted.success)
    {
      RCLCPP_WARN(node->get_logger(), "lift failed: %s", lifted.message.c_str());
      target++;
      continue;
    }

    RCLCPP_INFO(node->get_logger(),
                "leg %zu: %zu waypoints, planned in %.1f ms, max tube tilt %.3f rad",
                target, lifted.joints.size(), res.planning_time * 1e3, lifted.max_tilt);

    for (const auto & q7 : lifted.joints)
    {
      if (!rclcpp::ok()) break;
      publish(q7);
      rclcpp::spin_some(node);
      rate.sleep();
    }

    q_now = res.path.back();
    roll_a = roll_b;
    const auto until = node->now() + rclcpp::Duration::from_seconds(dwell);
    while (rclcpp::ok() && node->now() < until)
    {
      publish(lifted.joints.back());
      rclcpp::spin_some(node);
      rate.sleep();
    }
    target++;
  }

  rclcpp::shutdown();
  return 0;
}
