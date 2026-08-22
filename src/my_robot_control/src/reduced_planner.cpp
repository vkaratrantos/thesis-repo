#include "my_robot_control/reduced_planner.hpp"

#include "my_robot_control/path_lifter.hpp"

#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace my_robot_control
{
namespace
{

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

// ---------------------------------------------------------------------------

struct ReducedPlanner::Impl
{
  ob::StateSpacePtr space;
  og::SimpleSetupPtr setup;
  mutable std::atomic<std::size_t> checks{ 0 };

  // One RobotState per checker, reused. Allocating a RobotState per call would
  // dominate the runtime -- the collision check itself is the cheap part.
  mutable std::unique_ptr<moveit::core::RobotState> work_state;
};

ReducedPlanner::ReducedPlanner() : impl_(std::make_unique<Impl>()) {}
ReducedPlanner::~ReducedPlanner() = default;

// ---------------------------------------------------------------------------

void ReducedPlanner::alignBlob(moveit::core::RobotState & state) const
{
  // The blob is a solid of revolution about its own z. We want that z to point
  // along WORLD z, whatever the arm is doing, because the upright constraint
  // the blob was swept under is stated in the world frame.
  //
  // With the chain wrist_center -[Rx(a)]- blob_align_link -[Ry(b)]- wrist_blob,
  // the blob's z expressed in the wrist-centre frame is
  //
  //     Rx(a) Ry(b) e_z = ( sin b, -sin a cos b, cos a cos b )
  //
  // and we need that to equal v = R_wc^T * e_z_world. Reading off:
  //
  //     b = asin(v_x),   a = atan2(-v_y, v_z)
  //
  // The third rotation is about the symmetry axis, so two joints suffice.
  const Eigen::Matrix3d R_wc = state.getGlobalLinkTransform(opt_.wrist_centre_link).rotation();
  const Eigen::Vector3d v = R_wc.transpose() * Eigen::Vector3d::UnitZ();

  const double b = std::asin(std::clamp(v.x(), -1.0, 1.0));
  const double a = std::atan2(-v.y(), v.z());

  state.setJointPositions(opt_.align_joint_x, &a);
  state.setJointPositions(opt_.align_joint_y, &b);
}

// ---------------------------------------------------------------------------

void ReducedPlanner::initialize(const Options & options)
{
  opt_ = options;

  const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(readFile(opt_.urdf_path));
  if (!urdf) throw std::runtime_error("failed to parse " + opt_.urdf_path);

  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, readFile(opt_.srdf_path)))
    throw std::runtime_error("failed to parse " + opt_.srdf_path);

  model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

  jmg_ = model_->getJointModelGroup(opt_.group);
  if (!jmg_)
    throw std::runtime_error("reduced model has no group '" + opt_.group + "'");

  kin_ = std::make_unique<SrsKinematics>(model_, opt_.wrist_centre_link);

  // Map each state-space dimension onto its slot in q1..q4. The 3-DOF group
  // skips joint3, which is exactly the "lock the arm angle" variant.
  dim_to_arm_index_.clear();
  const std::vector<std::string> arm_names{ "joint1", "joint2", "joint3", "joint4" };
  for (const std::string & name : jmg_->getActiveJointModelNames())
  {
    const auto it = std::find(arm_names.begin(), arm_names.end(), name);
    if (it == arm_names.end())
      throw std::runtime_error("group '" + opt_.group + "' contains unexpected joint " + name);
    dim_to_arm_index_.push_back(static_cast<int>(std::distance(arm_names.begin(), it)));
  }
  const std::size_t dim = dim_to_arm_index_.size();

  // ---- state space -------------------------------------------------------
  auto space = std::make_shared<ob::RealVectorStateSpace>(dim);
  ob::RealVectorBounds bounds(dim);
  for (std::size_t d = 0; d < dim; ++d)
  {
    bounds.setLow(d, kin_->lower()[dim_to_arm_index_[d]]);
    bounds.setHigh(d, kin_->upper()[dim_to_arm_index_[d]]);
  }
  space->setBounds(bounds);
  space->setLongestValidSegmentFraction(opt_.longest_valid_segment_fraction);
  impl_->space = space;

  impl_->setup = std::make_shared<og::SimpleSetup>(space);
  impl_->work_state = std::make_unique<moveit::core::RobotState>(model_);
  impl_->work_state->setToDefaultValues();

  impl_->setup->setStateValidityChecker([this](const ob::State * s) {
    const auto * rv = s->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d q(0.0, 0.0, locked_q3_, 0.0);
    for (std::size_t d = 0; d < dim_to_arm_index_.size(); ++d)
      q[dim_to_arm_index_[d]] = rv->values[d];
    return isValid(q);
  });

  auto planner = std::make_shared<og::RRTConnect>(impl_->setup->getSpaceInformation());
  if (opt_.rng_seed != 0) ompl::RNG::setSeed(opt_.rng_seed);
  impl_->setup->setPlanner(planner);
}

// ---------------------------------------------------------------------------

void ReducedPlanner::syncWorld(const planning_scene::PlanningScene & live_scene,
                               const std::set<std::string> & ignore_ids)
{
  if (!scene_) throw std::runtime_error("ReducedPlanner::syncWorld before initialize()");

  scene_->getWorldNonConst()->clearObjects();

  const auto & live_world = live_scene.getWorld();
  for (const std::string & id : live_world->getObjectIds())
  {
    if (ignore_ids.count(id)) continue;
    const auto obj = live_world->getObject(id);
    if (!obj) continue;
    scene_->getWorldNonConst()->addToObject(id, obj->shapes_, obj->shape_poses_);
  }

  // Objects ATTACHED to the live robot matter too -- the carried tube is the
  // obvious one -- but it is already baked into the blob, so anything attached
  // is deliberately not copied. Only the world is transferred.
}

// ---------------------------------------------------------------------------

bool ReducedPlanner::isValid(const Eigen::Vector4d & q) const
{
  impl_->checks.fetch_add(1, std::memory_order_relaxed);

  moveit::core::RobotState & s = *impl_->work_state;
  const char * names[4] = { "joint1", "joint2", "joint3", "joint4" };
  for (int k = 0; k < 4; ++k) s.setJointPositions(names[k], &q[k]);
  s.update();

  alignBlob(s);
  s.update();

  if (!s.satisfiesBounds(jmg_)) return false;

  // group_name is left empty on purpose: the blob is not a member of the
  // planning group, and a group-scoped request would skip exactly the link we
  // added the whole model for.
  collision_detection::CollisionRequest req;
  req.contacts = false;
  collision_detection::CollisionResult res;
  scene_->checkCollision(req, res, s, scene_->getAllowedCollisionMatrix());
  return !res.collision;
}

// ---------------------------------------------------------------------------

ReducedPlanner::Result ReducedPlanner::solveFrom(
    const Eigen::Vector4d & start_q,
    const std::vector<Eigen::Vector4d> & goal_candidates,
    const std::string & empty_goal_message)
{
  Result r;
  const std::size_t dim = dim_to_arm_index_.size();

  // Timing starts here, not after the IK: turning the Cartesian goal into joint
  // goals is work the 7-DOF pipeline also has to do, so excluding it would
  // flatter this side of the benchmark.
  const auto t0 = std::chrono::steady_clock::now();
  impl_->checks.store(0);

  std::vector<Eigen::Vector4d> reachable;
  for (const auto & q : goal_candidates)
    if (isValid(q)) reachable.push_back(q);

  if (reachable.empty())
  {
    r.planning_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.states_checked = impl_->checks.load();
    r.message = goal_candidates.empty()
                    ? empty_goal_message
                    : "every IK solution for the goal is in collision (" +
                          std::to_string(goal_candidates.size()) + " tried)";
    return r;
  }

  auto & setup = *impl_->setup;
  const auto si = setup.getSpaceInformation();

  ob::ScopedState<> start(impl_->space);
  for (std::size_t d = 0; d < dim; ++d) start[d] = start_q[dim_to_arm_index_[d]];

  setup.clear();
  setup.setStartState(start);

  auto goals = std::make_shared<ob::GoalStates>(si);
  for (const auto & q : reachable)
  {
    ob::ScopedState<> g(impl_->space);
    for (std::size_t d = 0; d < dim; ++d) g[d] = q[dim_to_arm_index_[d]];
    goals->addState(g);
  }
  setup.setGoal(goals);

  const auto solved = setup.solve(opt_.planning_time);
  if (solved && setup.haveExactSolutionPath() && opt_.simplify) setup.simplifySolution();

  r.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  r.states_checked = impl_->checks.load();
  r.goal_states = reachable.size();

  if (!solved || !setup.haveExactSolutionPath())
  {
    r.message = "no exact solution within the time budget";
    return r;
  }

  auto path = setup.getSolutionPath();
  path.interpolate();
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    const auto * rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d q(0.0, 0.0, locked_q3_, 0.0);
    for (std::size_t d = 0; d < dim; ++d) q[dim_to_arm_index_[d]] = rv->values[d];
    r.path.push_back(q);
  }
  r.success = true;
  return r;
}

// ---------------------------------------------------------------------------

ReducedPlanner::Result ReducedPlanner::planToJointGoal(const Eigen::Vector4d & start_q,
                                                       const Eigen::Vector4d & goal_q)
{
  locked_q3_ = start_q[2];
  return solveFrom(start_q, { goal_q }, "goal configuration is invalid");
}

// ---------------------------------------------------------------------------

ReducedPlanner::Result ReducedPlanner::planToWristCentre(const Eigen::Vector4d & start_q,
                                                         const Eigen::Vector3d & goal_W)
{
  locked_q3_ = start_q[2];

  // One Cartesian goal maps to a whole family of joint configurations: two
  // elbow branches, two shoulder branches, and -- on the 4-DOF group -- a
  // continuum of arm angles. Handing OMPL all of them is most of why the goal
  // becomes easy to reach.
  const bool locked = (dim_to_arm_index_.size() == 3);
  const auto candidates = locked ? kin_->positionIk(goal_W, locked_q3_)
                                 : kin_->positionIkSampled(goal_W, opt_.arm_angle_samples);

  return solveFrom(start_q, candidates,
                   "wrist centre is out of reach or violates joint limits");
}

// ---------------------------------------------------------------------------

ReducedPlanner::Result ReducedPlanner::planToToolPose(const Eigen::Vector4d & start_q,
                                                      const Eigen::Vector3d & tcp_position,
                                                      double flange_offset,
                                                      int roll_samples)
{
  locked_q3_ = start_q[2];
  const bool locked = (dim_to_arm_index_.size() == 3);
  if (roll_samples < 1) roll_samples = 1;

  // With the tool axis vertical, the wrist-centre-to-flange offset is
  // horizontal, so W = tcp - flange_offset * (cos phi, sin phi, 0) for a free
  // roll phi. Every phi is an equally good way to satisfy the task.
  std::vector<Eigen::Vector4d> candidates;
  for (int i = 0; i < roll_samples; ++i)
  {
    const double phi = 2.0 * M_PI * i / roll_samples;
    const Eigen::Vector3d W = tcp_position - flange_offset * PathLifter::flangeDirection(phi);
    const auto sols = locked ? kin_->positionIk(W, locked_q3_)
                             : kin_->positionIkSampled(W, opt_.arm_angle_samples);
    candidates.insert(candidates.end(), sols.begin(), sols.end());
  }

  return solveFrom(start_q, candidates,
                   "no roll angle puts the wrist centre within reach of that TCP position");
}

}  // namespace my_robot_control
