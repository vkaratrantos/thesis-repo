#include "my_robot_control/manifold_planner.hpp"

#include <moveit/robot_state/robot_state.h>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace my_robot_control
{
namespace
{
const char * kJoints[7] = { "joint1", "joint2", "joint3", "joint4",
                            "joint5", "joint6", "joint7" };
}  // namespace

// Rejects an edge whose wrist cannot follow it continuously.
//
// q5 and q7 come from atan2 and wrap at +/-pi. Across that wrap the orientation
// is continuous but the joint value jumps 2*pi, and joint7's limit is exactly
// +/-pi, so a path crossing the seam asks it to rotate through the one point it
// cannot reach. Checking that at extraction time, after the search, only lets
// us reject the finished plan. Checking it HERE makes the seam part of the
// search: RRTConnect simply routes around it, because edges that cross it are
// not traversable.
class ManifoldMotionValidator : public ob::MotionValidator
{
public:
  ManifoldMotionValidator(const ob::SpaceInformationPtr & si, const ManifoldPlanner * owner)
  : ob::MotionValidator(si), owner_(owner) {}

  bool checkMotion(const ob::State * s1, const ob::State * s2) const override
  {
    std::pair<ob::State *, double> unused{ nullptr, 0.0 };
    return walk(s1, s2, unused, false);
  }

  bool checkMotion(const ob::State * s1, const ob::State * s2,
                   std::pair<ob::State *, double> & lastValid) const override
  {
    return walk(s1, s2, lastValid, true);
  }

private:
  bool config(const ob::State * s, ManifoldPlanner::JointVector & q) const
  {
    const auto * rv = s->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d arm(0.0, 0.0, owner_->lockedArmAngle(), 0.0);
    std::size_t d = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && owner_->lockArmAngle()) continue;
      arm[k] = rv->values[d++];
    }
    return owner_->stateToConfig(arm, rv->values[d], owner_->branch(), q);
  }

  bool walk(const ob::State * s1, const ob::State * s2,
            std::pair<ob::State *, double> & lastValid, bool want_last) const
  {
    ManifoldPlanner::JointVector prev, cur;
    if (!config(s1, prev) || !owner_->isValid(prev))
    {
      if (want_last) lastValid.second = 0.0;
      return false;
    }

    const int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
    if (nd <= 1) return true;

    ob::State * test = si_->allocState();
    bool ok = true;
    int j = 1;
    for (; j <= nd; ++j)
    {
      si_->getStateSpace()->interpolate(s1, s2, double(j) / double(nd), test);
      if (!config(test, cur) || !owner_->unwrapWrist(prev, cur) || !owner_->isValid(cur))
      {
        ok = false;
        break;
      }
      prev = cur;
    }

    if (!ok && want_last)
    {
      lastValid.second = double(j - 1) / double(nd);
      if (lastValid.first)
        si_->getStateSpace()->interpolate(s1, s2, lastValid.second, lastValid.first);
    }
    si_->freeState(test);
    if (ok) valid_++; else invalid_++;
    return ok;
  }

  const ManifoldPlanner * owner_;
};

struct ManifoldPlanner::Impl
{
  ob::StateSpacePtr space;
  og::SimpleSetupPtr setup;
  mutable std::atomic<std::size_t> checks{ 0 };
  mutable std::unique_ptr<moveit::core::RobotState> work;
};

ManifoldPlanner::ManifoldPlanner() : impl_(std::make_unique<Impl>()) {}
ManifoldPlanner::~ManifoldPlanner() = default;

// ---------------------------------------------------------------------------

void ManifoldPlanner::initialize(const Options & options)
{
  opt_ = options;
  if (!opt_.model) throw std::runtime_error("ManifoldPlanner: no robot model");

  kin_ = std::make_unique<SrsKinematics>(opt_.model, "link5");

  moveit::core::RobotState st(opt_.model);
  st.setToDefaultValues();
  for (const char * n : kJoints)
  {
    const double zero = 0.0;
    st.setJointPositions(n, &zero);
  }
  st.update();
  flange_offset_ = (st.getGlobalLinkTransform(opt_.tcp_link).translation() -
                    st.getGlobalLinkTransform("link5").translation())
                       .norm();

  // Dimensions: q1, q2, [q3], q4, phi
  const std::size_t dim = opt_.lock_arm_angle ? 4 : 5;
  auto space = std::make_shared<ob::RealVectorStateSpace>(dim);
  ob::RealVectorBounds b(dim);

  const int arm_idx[4] = { 0, 1, 2, 3 };
  std::size_t d = 0;
  for (int k : arm_idx)
  {
    if (k == 2 && opt_.lock_arm_angle) continue;
    b.setLow(d, kin_->lower()[k]);
    b.setHigh(d, kin_->upper()[k]);
    ++d;
  }
  // The roll is periodic. Bounding it to [-pi, pi] on a RealVector space means
  // the sampler will not wrap around, so a path that would naturally cross the
  // seam has to go the long way. Acceptable: the goal set spans the whole
  // circle anyway, so some goal is always reachable without wrapping.
  b.setLow(d, -M_PI);
  b.setHigh(d, M_PI);
  space->setBounds(b);
  space->setLongestValidSegmentFraction(opt_.longest_valid_segment_fraction);
  impl_->space = space;

  impl_->setup = std::make_shared<og::SimpleSetup>(space);
  impl_->work = std::make_unique<moveit::core::RobotState>(opt_.model);
  impl_->work->setToDefaultValues();

  impl_->setup->setStateValidityChecker([this](const ob::State * s) {
    const auto * rv = s->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d arm(0.0, 0.0, locked_q3_, 0.0);
    std::size_t d2 = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && opt_.lock_arm_angle) continue;
      arm[k] = rv->values[d2++];
    }
    const double phi = rv->values[d2];

    JointVector q;
    if (!stateToConfig(arm, phi, branch_, q)) return false;
    return isValid(q);
  });

  impl_->setup->getSpaceInformation()->setMotionValidator(
      std::make_shared<ManifoldMotionValidator>(impl_->setup->getSpaceInformation(), this));

  auto planner = std::make_shared<og::RRTConnect>(impl_->setup->getSpaceInformation());
  if (opt_.rng_seed != 0) ompl::RNG::setSeed(opt_.rng_seed);
  impl_->setup->setPlanner(planner);
}

// ---------------------------------------------------------------------------

void ManifoldPlanner::setScene(const planning_scene::PlanningSceneConstPtr & scene,
                               const moveit::core::RobotState & reference_state)
{
  scene_ = scene;
  // Copy the reference so gripper joints and attached bodies (the carried tube)
  // come along. Only the seven arm joints get overwritten per state.
  impl_->work = std::make_unique<moveit::core::RobotState>(reference_state);
}

// ---------------------------------------------------------------------------

bool ManifoldPlanner::stateToConfig(const Eigen::Vector4d & arm, double phi, int branch,
                                    JointVector & out) const
{
  for (int k = 0; k < 4; ++k)
    if (arm[k] < kin_->lower()[k] - 1e-9 || arm[k] > kin_->upper()[k] + 1e-9) return false;

  // wristIkBranch, not wristIk: the branch index has to mean the same thing at
  // every state, or the state->configuration map is discontinuous and the
  // solution path jumps between wrist postures mid-edge.
  Eigen::Vector3d w;
  if (!kin_->wristIkBranch(arm, PathLifter::uprightOrientation(phi), branch, w))
    return false;

  out.head<4>() = arm;
  out.tail<3>() = w;
  return true;
}

double ManifoldPlanner::limitHeadroom(const JointVector & q) const
{
  const auto & model = *opt_.model;
  double worst = std::numeric_limits<double>::max();

  for (int k = 0; k < 7; ++k)
  {
    const auto * jm = model.getJointModel(kJoints[k]);
    if (!jm || jm->getVariableBoundsMsg().empty()) continue;
    const auto & b = jm->getVariableBoundsMsg().front();
    if (!b.has_position_limits) continue;
    worst = std::min(worst, std::min(q[k] - b.min_position, b.max_position - q[k]));
  }
  return (worst == std::numeric_limits<double>::max()) ? 0.0 : worst;
}

bool ManifoldPlanner::isValid(const JointVector & q) const
{
  impl_->checks.fetch_add(1, std::memory_order_relaxed);
  if (!scene_) return false;

  moveit::core::RobotState & s = *impl_->work;
  for (int k = 0; k < 7; ++k) s.setJointPositions(kJoints[k], &q[k]);
  s.update();

  if (!s.satisfiesBounds()) return false;

  collision_detection::CollisionRequest req;
  req.contacts = false;
  collision_detection::CollisionResult res;
  scene_->checkCollision(req, res, s, scene_->getAllowedCollisionMatrix());
  return !res.collision;
}

// ---------------------------------------------------------------------------

ManifoldPlanner::Result ManifoldPlanner::planToConfiguration(const JointVector & start_q,
                                                             const JointVector & goal_q)
{
  Result r;

  // Where does this configuration sit on the manifold? Read the roll off its
  // own forward kinematics, then find the wrist branch that reproduces it.
  moveit::core::RobotState s(*impl_->work);
  for (int k = 0; k < 7; ++k) s.setJointPositions(kJoints[k], &goal_q[k]);
  s.update();
  const Eigen::Vector3d tool = s.getGlobalLinkTransform(opt_.tcp_link).rotation().col(0);
  const double tilt = std::atan2(std::hypot(tool.x(), tool.y()), tool.z());
  if (tilt > 0.05)
  {
    r.message = "goal configuration is not upright (" + std::to_string(tilt) + " rad)";
    return r;
  }

  const double phi = PathLifter::rollFor(
      kin_->wristCentre(goal_q.head<4>()),
      s.getGlobalLinkTransform(opt_.tcp_link).translation());

  int goal_branch = -1;
  JointVector check;
  for (int b : { 0, 1 })
  {
    if (!stateToConfig(goal_q.head<4>(), phi, b, check)) continue;
    if ((check.tail<3>() - goal_q.tail<3>()).cwiseAbs().maxCoeff() < 1e-3)
    {
      goal_branch = b;
      break;
    }
  }
  if (goal_branch < 0)
  {
    r.message = "goal configuration does not lie on either wrist branch";
    return r;
  }

  // The start must be on the same branch, or the map is discontinuous.
  branch_ = goal_branch;
  locked_q3_ = start_q[2];

  const auto t0 = std::chrono::steady_clock::now();
  impl_->checks.store(0);

  if (!isValid(goal_q))
  {
    r.planning_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.message = "goal configuration is in collision";
    return r;
  }

  auto & setup = *impl_->setup;
  const std::size_t dim = opt_.lock_arm_angle ? 4 : 5;
  const auto pack = [&](const Eigen::Vector4d & arm, double p, ob::ScopedState<> & st) {
    std::size_t d = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && opt_.lock_arm_angle) continue;
      st[d++] = arm[k];
    }
    st[d] = p;
  };

  ob::ScopedState<> start(impl_->space), goal(impl_->space);
  {
    moveit::core::RobotState s0(*impl_->work);
    for (int k = 0; k < 7; ++k) s0.setJointPositions(kJoints[k], &start_q[k]);
    s0.update();
    pack(start_q.head<4>(),
         PathLifter::rollFor(kin_->wristCentre(start_q.head<4>()),
                             s0.getGlobalLinkTransform(opt_.tcp_link).translation()),
         start);
  }
  pack(goal_q.head<4>(), phi, goal);

  setup.clear();
  setup.setStartState(start);
  auto gs = std::make_shared<ob::GoalStates>(setup.getSpaceInformation());
  gs->addState(goal);
  setup.setGoal(gs);

  const auto solved = setup.solve(opt_.planning_time);
  if (solved && setup.haveExactSolutionPath() && opt_.simplify) setup.simplifySolution();

  r.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  r.states_checked = impl_->checks.load();
  r.goal_states = 1;
  r.branch_used = goal_branch;

  if (!solved || !setup.haveExactSolutionPath())
  {
    r.message = "no exact solution to the goal configuration";
    return r;
  }

  auto path = setup.getSolutionPath();
  path.interpolate();
  moveit::core::RobotState w(*impl_->work);
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    const auto * rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d arm(0.0, 0.0, locked_q3_, 0.0);
    std::size_t d = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && opt_.lock_arm_angle) continue;
      arm[k] = rv->values[d++];
    }
    JointVector q;
    if (!stateToConfig(arm, rv->values[d], goal_branch, q))
    {
      r.message = "solution path contains a state with no wrist solution";
      return r;
    }
    if (!r.path.empty())
    {
      const JointVector prev = r.path.back();
      if (!unwrapWrist(prev, q))
      {
        r.message = "solution path needs a wrist rotation past a joint limit";
        return r;
      }
      r.max_joint_step = std::max(r.max_joint_step, (q - prev).cwiseAbs().maxCoeff());
    }
    r.path.push_back(q);

    for (int k = 0; k < 7; ++k) w.setJointPositions(kJoints[k], &q[k]);
    w.update();
    const Eigen::Vector3d ax = w.getGlobalLinkTransform(opt_.tcp_link).rotation().col(0);
    r.max_tilt = std::max(r.max_tilt, std::atan2(std::hypot(ax.x(), ax.y()), ax.z()));
  }
  (void)dim;
  r.success = true;
  return r;
}

bool ManifoldPlanner::unwrapWrist(const JointVector & prev, JointVector & q) const
{
  for (int k = 4; k < 7; ++k)
  {
    while (q[k] - prev[k] > M_PI) q[k] -= 2.0 * M_PI;
    while (q[k] - prev[k] < -M_PI) q[k] += 2.0 * M_PI;
    if (q[k] < kin_->lower()[k] - 1e-9 || q[k] > kin_->upper()[k] + 1e-9) return false;
  }
  return true;
}

ManifoldPlanner::Result ManifoldPlanner::planOnBranch(const JointVector & start_q,
                                                      const Eigen::Vector3d & tcp,
                                                      int branch,
                                                      const double * required_roll,
                                                      const JointVector * prefer)
{
  Result r;
  r.branch_used = branch;
  branch_ = branch;
  locked_q3_ = start_q[2];

  const std::size_t dim = opt_.lock_arm_angle ? 4 : 5;
  auto & setup = *impl_->setup;
  const auto si = setup.getSpaceInformation();

  const auto pack = [&](const Eigen::Vector4d & arm, double phi, ob::ScopedState<> & s) {
    std::size_t d = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && opt_.lock_arm_angle) continue;
      s[d++] = arm[k];
    }
    s[d] = phi;
  };

  const auto t0 = std::chrono::steady_clock::now();
  impl_->checks.store(0);

  // ---- goal set ---------------------------------------------------------
  // With the tool upright the wrist-centre-to-flange offset is horizontal, so
  // W = tcp - off * (cos phi, sin phi, 0). Sampling phi and the arm angle gives
  // a large goal set, which is most of why the goal is easy to reach.
  // Candidates carry their distance to the preferred configuration so the hint
  // can rank them afterwards instead of eliminating them as they are found.
  struct GoalCandidate
  {
    Eigen::Vector4d arm;
    double phi;
    double hint_distance;   // max per-joint distance to `prefer`, or 0 without one
  };
  std::vector<GoalCandidate> candidates;
  std::size_t n_arm_ik = 0, n_no_wrist = 0, n_collide = 0, n_far = 0, n_pinned = 0;

  // Rolls to try. Free roll sweeps the circle; a pinned roll still gets the
  // goal-orientation tolerance, same as the baseline.
  // When the caller named a preferred configuration it is because precision
  // matters -- approachTarget does it before a straight-line descent onto a
  // tube. There the goal tolerance is actively harmful: 0.02 rad of roll swings
  // the fingertips 0.201 * 0.02 = 4 mm, which stacks with the 5 mm position
  // slack, and ~9 mm is enough to land the fingers ON the tube instead of
  // around it. The descent then stalls a tenth of the way down. So when a hint
  // is given, hit the pose exactly.
  const bool exact = (prefer != nullptr);

  std::vector<double> rolls;
  if (required_roll)
  {
    rolls.push_back(*required_roll);
    if (!exact && opt_.goal_roll_tolerance > 0.0)
    {
      rolls.push_back(*required_roll - opt_.goal_roll_tolerance);
      rolls.push_back(*required_roll + opt_.goal_roll_tolerance);
    }
  }
  else
  {
    for (int i = 0; i < opt_.roll_samples; ++i)
      rolls.push_back(-M_PI + 2.0 * M_PI * i / opt_.roll_samples);
  }

  // TCP offsets within the goal position tolerance. The centre first, so an
  // exact solution is always preferred when one exists.
  std::vector<Eigen::Vector3d> offsets{ Eigen::Vector3d::Zero() };
  if (!exact && opt_.goal_position_tolerance > 0.0)
  {
    const double t = opt_.goal_position_tolerance;
    for (int axis = 0; axis < 3; ++axis)
      for (const double sgn : { -1.0, 1.0 })
      {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        d[axis] = sgn * t;
        offsets.push_back(d);
      }
  }

  for (const double phi : rolls)
  for (const Eigen::Vector3d & d : offsets)
  {
    const Eigen::Vector3d W =
        (tcp + d) - flange_offset_ * PathLifter::flangeDirection(phi);
    const auto arms = opt_.lock_arm_angle
                          ? kin_->positionIk(W, locked_q3_)
                          : kin_->positionIkSampled(W, opt_.arm_angle_samples);
    n_arm_ik += arms.size();
    for (const auto & a : arms)
    {
      JointVector q;
      if (!stateToConfig(a, phi, branch, q)) { ++n_no_wrist; continue; }
      if (!isValid(q)) { ++n_collide; continue; }

      // A goal pressed against a joint stop is rejected outright, hint or no
      // hint. Unlike the hint this is not a preference: there is nowhere to go
      // from such a configuration, so reaching it accomplishes nothing.
      if (limitHeadroom(q) < opt_.goal_limit_headroom) { ++n_pinned; continue; }

      const double dist = prefer ? (q - *prefer).cwiseAbs().maxCoeff() : 0.0;
      if (prefer && dist > opt_.goal_hint_radius) ++n_far;
      candidates.push_back({ a, phi, dist });
    }
  }

  // Apply the hint as a ranking. Candidates inside the radius win; if none are,
  // the closest few are kept rather than failing -- failing here would hand the
  // caller's fallback a problem with no preference expressed at all.
  std::vector<std::pair<Eigen::Vector4d, double>> goals;
  if (prefer && !candidates.empty())
  {
    std::sort(candidates.begin(), candidates.end(),
              [](const GoalCandidate & a, const GoalCandidate & b)
              { return a.hint_distance < b.hint_distance; });

    std::size_t keep = 0;
    while (keep < candidates.size() &&
           candidates[keep].hint_distance <= opt_.goal_hint_radius)
      ++keep;

    if (keep == 0)
      keep = std::min(candidates.size(), opt_.goal_hint_fallback_count);

    for (std::size_t i = 0; i < keep; ++i)
      goals.emplace_back(candidates[i].arm, candidates[i].phi);
  }
  else
  {
    for (const auto & c : candidates) goals.emplace_back(c.arm, c.phi);
  }

  if (goals.empty())
  {
    r.planning_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.states_checked = impl_->checks.load();
    // Say WHY. "0 goals" is useless on its own: arm IK finding nothing means
    // the position is out of reach, the wrist branch failing means a joint
    // limit, and everything colliding means the scene is in the way. Those
    // need completely different fixes.
    r.message = "no goal configuration: " + std::to_string(n_arm_ik) +
                " arm IK solutions, " + std::to_string(n_no_wrist) +
                " rejected by wrist limits, " + std::to_string(n_collide) +
                " in collision, " + std::to_string(n_pinned) +
                " pinned against a joint stop, " + std::to_string(n_far) +
                " outside the hint radius (kept anyway if nothing was inside it)";
    return r;
  }

  ob::ScopedState<> start(impl_->space);
  {
    const Eigen::Vector3d W = kin_->wristCentre(start_q.head<4>());
    moveit::core::RobotState s(*impl_->work);
    for (int k = 0; k < 7; ++k) s.setJointPositions(kJoints[k], &start_q[k]);
    s.update();
    const Eigen::Vector3d tcp_now = s.getGlobalLinkTransform(opt_.tcp_link).translation();
    pack(start_q.head<4>(), PathLifter::rollFor(W, tcp_now), start);
  }

  setup.clear();
  setup.setStartState(start);

  auto gs = std::make_shared<ob::GoalStates>(si);
  for (const auto & [arm, phi] : goals)
  {
    ob::ScopedState<> g(impl_->space);
    pack(arm, phi, g);
    gs->addState(g);
  }
  setup.setGoal(gs);

  const auto solved = setup.solve(opt_.planning_time);
  if (solved && setup.haveExactSolutionPath() && opt_.simplify) setup.simplifySolution();

  r.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  r.states_checked = impl_->checks.load();
  r.goal_states = goals.size();

  if (!solved || !setup.haveExactSolutionPath())
  {
    r.message = std::string("planner status: ") + solved.asString() +
                (setup.haveSolutionPath() ? " (approximate only)" : " (no path)");
    return r;
  }

  auto path = setup.getSolutionPath();
  path.interpolate();

  moveit::core::RobotState s(*impl_->work);
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    const auto * rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector4d arm(0.0, 0.0, locked_q3_, 0.0);
    std::size_t d = 0;
    for (int k : { 0, 1, 2, 3 })
    {
      if (k == 2 && opt_.lock_arm_angle) continue;
      arm[k] = rv->values[d++];
    }
    JointVector q;
    if (!stateToConfig(arm, rv->values[d], branch, q))
    {
      r.message = "solution path contains a state with no wrist solution";
      return r;
    }

    // Unwrap the wrist joints against the previous waypoint.
    //
    // q5 and q7 come out of atan2, so they live in (-pi, pi] and WRAP. The
    // orientation is perfectly continuous across that wrap -- Rz(q) and
    // Rz(q - 2pi) are the same rotation -- but the joint VALUE jumps by 2pi,
    // and a controller replaying those waypoints executes a full 360 degree
    // spin of the wrist. That sweeps the tube through every orientation on the
    // way, which is exactly the transient tilt spike seen on the real
    // trajectory while the planner reported ~0 tilt: the planner was right
    // about every waypoint and wrong about the motion between two of them.
    if (!r.path.empty())
    {
      const JointVector prev = r.path.back();
      // Safety net. The motion validator should already have kept the search
      // clear of the seam, so this must not fire.
      if (!unwrapWrist(prev, q))
      {
        r.message = "solution path needs a wrist rotation past a joint limit";
        return r;
      }
      r.max_joint_step = std::max(r.max_joint_step,
                                  (q - prev).cwiseAbs().maxCoeff());
    }

    r.path.push_back(q);

    for (int k = 0; k < 7; ++k) s.setJointPositions(kJoints[k], &q[k]);
    s.update();
    const Eigen::Vector3d axis = s.getGlobalLinkTransform(opt_.tcp_link).rotation().col(0);
    r.max_tilt = std::max(r.max_tilt,
                          std::atan2(std::hypot(axis.x(), axis.y()), axis.z()));
  }

  r.success = true;
  (void)dim;
  return r;
}

ManifoldPlanner::Result ManifoldPlanner::planToToolPose(const JointVector & start_q,
                                                        const Eigen::Vector3d & tcp,
                                                        const double * required_roll,
                                                        const JointVector * prefer)
{
  // Which branch is the arm already on? Starting there means the plan begins at
  // the configuration the robot is actually in, with no jump.
  //
  // This must be evaluated at the start's ACTUAL roll. Comparing against
  // upright(0) instead compares the arm to a posture it is not in, picks the
  // wrong branch, and the start state then fails its own validity check --
  // which shows up as "RRTConnect: start tree could not be initialized".
  int first = 0;
  {
    moveit::core::RobotState s(*impl_->work);
    for (int k = 0; k < 7; ++k) s.setJointPositions(kJoints[k], &start_q[k]);
    s.update();
    const double phi0 = PathLifter::rollFor(
        kin_->wristCentre(start_q.head<4>()),
        s.getGlobalLinkTransform(opt_.tcp_link).translation());

    const auto R0 = PathLifter::uprightOrientation(phi0);
    double best = std::numeric_limits<double>::infinity();
    for (int br : { 0, 1 })
    {
      Eigen::Vector3d w;
      if (!kin_->wristIkBranch(start_q.head<4>(), R0, br, w)) continue;
      const double d = (w - start_q.tail<3>()).cwiseAbs().sum();
      if (d < best) { best = d; first = br; }
    }
  }

  Result r = planOnBranch(start_q, tcp, first, required_roll, prefer);
  if (r.success) return r;

  // Deliberately NO retry without the hint.
  //
  // Widening the goal set when the hint cannot be met looks generous and is
  // actively harmful: it finds a configuration at the right pose that the
  // caller's follow-up Cartesian move cannot start from, so the arm arrives
  // above the tube and then stalls partway down. Measured directly -- goal sets
  // of 627 states failed the descent at 23-38%, the hint-filtered set of 1
  // state completed it at 100%.
  //
  // Failing here instead hands the move to moveFreeSpaceUpright, which lands in
  // a configuration the descent does work from.
  if (r.success) return r;

  Result other = planOnBranch(start_q, tcp, 1 - first, required_roll, prefer);
  other.planning_time += r.planning_time;
  other.states_checked += r.states_checked;

  // Carry the FIRST branch's reason forward even when the second one succeeds.
  // The second branch starts a wrist flip away from where the arm actually is,
  // so the caller has to bridge that gap or refuse -- and it can only judge
  // which when it knows why the arm's own branch was rejected.
  other.message = "branch " + std::to_string(first) + " (the arm's own): " +
                  r.message +
                  (other.success ? ""
                                 : ("; branch " + std::to_string(1 - first) + ": " +
                                    other.message));
  return other;
}

bool ManifoldPlanner::rollOfOrientation(const Eigen::Matrix3d & R, double & roll, double tol)
{
  // Upright means the tool's x axis -- the tube's long axis -- points at world
  // +z. Check that before trusting the roll we read off the z axis.
  const Eigen::Vector3d x = R.col(0);
  if (std::atan2(std::hypot(x.x(), x.y()), x.z()) > tol) return false;

  // The flange offset lies along the tool's z, which flangeDirection() says is
  // (sin phi, -cos phi, 0).
  const Eigen::Vector3d z = R.col(2);
  roll = std::atan2(z.x(), -z.y());
  return true;
}

}  // namespace my_robot_control
