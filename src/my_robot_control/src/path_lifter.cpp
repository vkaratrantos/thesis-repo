#include "my_robot_control/path_lifter.hpp"

#include <moveit/robot_state/robot_state.h>

#include <cmath>
#include <stdexcept>

namespace my_robot_control
{
namespace
{
const char * kArmJoints[7] = { "joint1", "joint2", "joint3",
                               "joint4", "joint5", "joint6", "joint7" };
}  // namespace

PathLifter::PathLifter(const moveit::core::RobotModelConstPtr & full_model,
                       const std::string & tcp_link)
: model_(full_model), tcp_link_(tcp_link)
{
  if (!model_) throw std::runtime_error("PathLifter: null robot model");
  for (const char * n : kArmJoints)
    if (!model_->hasJointModel(n))
      throw std::runtime_error(std::string("PathLifter: full model needs ") + n);

  // link5's frame origin is the wrist centre -- joints 5, 6 and 7 all pass
  // through it, which is what makes the wrist spherical.
  kin_ = std::make_unique<SrsKinematics>(model_, "link5");

  // Wrist centre to flange, measured off the model rather than hardcoded.
  moveit::core::RobotState st(model_);
  st.setToDefaultValues();
  for (const char * n : kArmJoints)
  {
    const double zero = 0.0;
    st.setJointPositions(n, &zero);
  }
  st.update();
  flange_offset_ = (st.getGlobalLinkTransform(tcp_link_).translation() -
                    st.getGlobalLinkTransform("link5").translation())
                       .norm();
}

Eigen::Matrix3d PathLifter::uprightOrientation(double phi)
{
  // q_upright = RPY(0, -pi/2, pi/2) from simple_move.cpp. Its x axis -- the
  // tube's long axis -- points at world +z, which is what keeps the liquid in.
  // Rolling about the world vertical leaves that property intact.
  const Eigen::Matrix3d base =
      (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  return Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).toRotationMatrix() * base;
}

Eigen::Vector3d PathLifter::flangeDirection(double phi)
{
  return { std::sin(phi), -std::cos(phi), 0.0 };
}

double PathLifter::rollFor(const Eigen::Vector3d & wrist_centre,
                           const Eigen::Vector3d & tcp_position)
{
  const Eigen::Vector3d d = tcp_position - wrist_centre;
  return std::atan2(d.x(), -d.y());
}

double PathLifter::rollOf(const moveit::core::RobotState & state) const
{
  const Eigen::Vector3d W = state.getGlobalLinkTransform("link5").translation();
  const Eigen::Vector3d tcp = state.getGlobalLinkTransform(tcp_link_).translation();
  return rollFor(W, tcp);
}

PathLifter::Result PathLifter::lift(const std::vector<Eigen::Vector4d> & wrist_path,
                                    double roll_start,
                                    double roll_end) const
{
  Result r;
  if (wrist_path.empty())
  {
    r.message = "empty wrist path";
    return r;
  }

  // Interpolate roll along the SHORTER arc. Going the long way round would spin
  // the gripper through most of a turn for no reason, and on this arm the tube
  // sits 201 mm off the wrist centre, so that is 400 mm of pointless travel.
  double delta = std::remainder(roll_end - roll_start, 2.0 * M_PI);

  Eigen::Vector3d seed = Eigen::Vector3d::Zero();
  r.joints.reserve(wrist_path.size());

  for (std::size_t i = 0; i < wrist_path.size(); ++i)
  {
    const double a = (wrist_path.size() == 1)
                         ? 1.0
                         : static_cast<double>(i) / (wrist_path.size() - 1);
    const double phi = roll_start + a * delta;

    JointVector q7;
    if (!kin_->lift(wrist_path[i], uprightOrientation(phi), seed, q7))
    {
      r.first_failure = i;
      r.message = "no wrist solution within joint limits at waypoint " + std::to_string(i);
      return r;
    }
    seed = q7.tail<3>();
    r.joints.push_back(q7);
  }

  // Report the achieved tilt rather than assuming it. The wrist IK is exact, so
  // this should be ~0; if it ever is not, something upstream is wrong and the
  // caller should see it.
  moveit::core::RobotState st(model_);
  st.setToDefaultValues();
  for (const auto & q : r.joints)
  {
    for (int k = 0; k < 7; ++k) st.setJointPositions(kArmJoints[k], &q[k]);
    st.update();
    const Eigen::Vector3d tool_axis = st.getGlobalLinkTransform(tcp_link_).rotation().col(0);
    r.max_tilt = std::max(r.max_tilt,
                          std::atan2(std::hypot(tool_axis.x(), tool_axis.y()), tool_axis.z()));
  }

  r.success = true;
  return r;
}

bool PathLifter::validate(const planning_scene::PlanningScene & scene,
                          const moveit::core::RobotState & reference_state,
                          const std::vector<JointVector> & joints,
                          std::size_t & first_bad) const
{
  // Start from the live state so the gripper joints and, critically, any
  // attached objects (the carried tube) come along. Only the arm is overridden.
  moveit::core::RobotState st(reference_state);

  collision_detection::CollisionRequest req;
  req.contacts = false;

  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    for (int k = 0; k < 7; ++k) st.setJointPositions(kArmJoints[k], &joints[i][k]);
    st.update();

    if (!st.satisfiesBounds())
    {
      first_bad = i;
      return false;
    }

    collision_detection::CollisionResult res;
    scene.checkCollision(req, res, st, scene.getAllowedCollisionMatrix());
    if (res.collision)
    {
      first_bad = i;
      return false;
    }
  }
  return true;
}

}  // namespace my_robot_control
