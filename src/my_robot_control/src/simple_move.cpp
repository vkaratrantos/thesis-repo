// ============================================================================
//  LIQUID HANDLING 7-DOF ROBOTIC ARM
// ============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "my_robot_control/manifold_planner.hpp"
#include "my_robot_control/path_lifter.hpp"

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // tf2::toMsg
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;
using MGI = moveit::planning_interface::MoveGroupInterface;

//  CONSTANTS

static constexpr double VEL_SCALE_TRANSIT = 0.30;
static constexpr double ACC_SCALE_TRANSIT = 0.30;
static constexpr double VEL_SCALE_LIQUID  = 0.15;   // slower when carrying
static constexpr double ACC_SCALE_LIQUID  = 0.15;

// ---- Table -----------------------------------------------------------------
static constexpr double TABLE_SIZE_XY   = 1.5;
static constexpr double TABLE_THICKNESS = 0.04;
static constexpr double TABLE_CENTER_Z  = -0.03;    // -> top surface at z = -0.01

// ---- Test tubes ------------------------------------------------------------
static constexpr double TUBE_HEIGHT = 0.13;
static constexpr double TUBE_RADIUS = 0.012;
static constexpr double TUBE_CENTER_Z = 0.06;
static constexpr double TUBE_Y_OFFSET = 0.02;

// ---- Mixing Vessel ---------------------------------------------------------
static constexpr double MIXER_HEIGHT   = 0.17;
static constexpr double MIXER_RADIUS   = 0.06;  
static constexpr double MIXER_CENTER_Z = 0.06;     
static constexpr double MIXER_Y_OFFSET = 0.10;

// ---- Grasp geometry --------------------------------------------------------
static constexpr double GRASP_Y_OFFSET = 0.155;
static constexpr double GRASP_Z        = 0.1;      // TCP height at the tube
static constexpr double POUR_Z         = 0.20;      // TCP height above the mixer
static constexpr double APPROACH_DIST  = 0.10;      // vertical descent to grasp
static constexpr double LIFT_DIST      = 0.12;      // vertical retreat after grasp
static constexpr double STANDOFF_TUBE  = 0.10;

// (The orientation CONSTRAINT still frees X)
static constexpr double TOOL_AXIS_FREEDOM = 0.0;
static constexpr double LIMIT_HEADROOM_MIN = 0.05;

// ---- Upright transits with an empty gripper --------------------------------
// ON by default
static std::atomic<bool> g_upright_when_empty{true};

// How far the tube may tip from vertical during a carrying move, in rad
static constexpr double TILT_TOLERANCE = 0.3;
static constexpr double STANDOFF_MIXER = 0.12;

// ---- Misc ------------------------------------------------------------------
// ---- Where the carried tube sits relative to the TCP ----------------------

static constexpr double TUBE_TCP_Y_OFFSET = -0.135;
static constexpr double TUBE_TCP_Z_OFFSET = -0.01;

static constexpr double MIXER_TOP_Z = MIXER_CENTER_Z + MIXER_HEIGHT / 2.0;

// ---- Pour pose: where the TCP goes, relative to marker_6 ------------------

static constexpr double POUR_X_OFFSET = 0.00;   // TCP x = marker_x + POUR_X_OFFSET
static constexpr double POUR_Y_OFFSET = 0.00;   // TCP y = marker_y + POUR_Y_OFFSET

// ---- Fallback positions, used only when TF has nothing -------------------

static constexpr int NUM_TUBES = 5;
static const double  TUBE_FALLBACK_X[NUM_TUBES] = { -0.17, -0.10, 0.00, 0.10, 0.17 };
static constexpr double TUBE_FALLBACK_Y  = -0.21;
static constexpr double TUBE_FALLBACK_Z  = 0.08;
static constexpr double MIXER_FALLBACK_MARKER_X = -0.20;
static constexpr double MIXER_FALLBACK_MARKER_Y = -0.11;

// Where to park the tube COLLISION OBJECTS when there is no TF
static const double  TUBE_OBJ_FALLBACK_X[NUM_TUBES] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
static constexpr double TUBE_OBJ_FALLBACK_Y = -0.335;
static constexpr double MIXER_OBJ_FALLBACK_X = -0.32;
static constexpr double MIXER_OBJ_FALLBACK_Y = -0.33;

static constexpr int MIXER_MARKER    = 6;
static constexpr int STATE_SETTLE_MS = 500;

//  SHARED STATE

static std::queue<std::string> command_queue;
static std::mutex              queue_mutex;

// Which tube is currently on the gripper

static std::atomic<int> attached_marker_id{ -1 };

// Hard pause for the background scene updater

static std::atomic<bool> scene_updates_paused{ false };

class ScenePause
{
public:
    ScenePause() { scene_updates_paused.store(true); }
    ~ScenePause() { scene_updates_paused.store(false); }
};

//  SCENE HELPERS

static bool lookupMarkerXY(tf2_ros::Buffer & tf_buffer,
                           const std::string & planning_frame,
                           int marker_id,
                           double & x,
                           double & y)
{
    try
    {
        const std::string marker_frame = "marker_" + std::to_string(marker_id);
        const auto t = tf_buffer.lookupTransform(planning_frame, marker_frame, tf2::TimePointZero);
        x = t.transform.translation.x;
        y = t.transform.translation.y;
        return true;
    }
    catch (const tf2::TransformException &)
    {
        return false;
    }
}

static moveit_msgs::msg::CollisionObject makeTube(int index,
                                                  const std::string & frame_id,
                                                  double marker_x,
                                                  double marker_y)
{
    moveit_msgs::msg::CollisionObject tube;
    tube.id = "tube_" + std::to_string(index);
    tube.header.frame_id = frame_id;
    tube.primitives.resize(1);
    tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    tube.primitives[0].dimensions = { TUBE_HEIGHT, TUBE_RADIUS };
    tube.primitive_poses.resize(1);
    tube.primitive_poses[0].position.x = marker_x;
    tube.primitive_poses[0].position.y = marker_y + TUBE_Y_OFFSET;
    tube.primitive_poses[0].position.z = TUBE_CENTER_Z;
    tube.primitive_poses[0].orientation.w = 1.0;
    tube.operation = tube.ADD;
    return tube;
}

static moveit_msgs::msg::CollisionObject makeMixer(const std::string & frame_id,
                                                   double marker_x,
                                                   double marker_y)
{
    moveit_msgs::msg::CollisionObject mixer;
    mixer.id = "mixer";
    mixer.header.frame_id = frame_id;
    mixer.primitives.resize(1);
    mixer.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    mixer.primitives[0].dimensions = { MIXER_HEIGHT, MIXER_RADIUS };
    mixer.primitive_poses.resize(1);
    mixer.primitive_poses[0].position.x = marker_x;
    mixer.primitive_poses[0].position.y = marker_y + MIXER_Y_OFFSET;
    mixer.primitive_poses[0].position.z = MIXER_CENTER_Z;
    mixer.primitive_poses[0].orientation.w = 1.0;
    mixer.operation = mixer.ADD;
    return mixer;
}

static void setupCollisionObjects(const std::string & frame_id, tf2_ros::Buffer & tf_buffer)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // ---- Table -------------------------------------------------------------
    moveit_msgs::msg::CollisionObject table;
    table.id = "table_base";
    table.header.frame_id = frame_id;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { TABLE_SIZE_XY, TABLE_SIZE_XY, TABLE_THICKNESS };
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.z = TABLE_CENTER_Z;
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;
    objects.push_back(table);

    // ---- Static obstacle 1 -------------------------------------------------
    moveit_msgs::msg::CollisionObject wall;
    wall.id = "obstacle_box";
    wall.header.frame_id = frame_id;
    wall.primitives.resize(1);
    wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall.primitives[0].dimensions = { 0.7, 0.2, 0.05 };
    wall.primitive_poses.resize(1);
    wall.primitive_poses[0].position.x = 0.0;
    wall.primitive_poses[0].position.y = 0.18;
    wall.primitive_poses[0].position.z = 0.02;
    wall.primitive_poses[0].orientation.w = 1.0;
    wall.operation = wall.ADD;
    objects.push_back(wall);

    // ---- Static obstacle 2 -------------------------------------------------
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "obstacle_box_2";
    wall2.header.frame_id = frame_id;
    wall2.primitives.resize(1);
    wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall2.primitives[0].dimensions = { 0.4, 0.2, 0.3 };
    wall2.primitive_poses.resize(1);
    wall2.primitive_poses[0].position.x = -0.6;
    wall2.primitive_poses[0].position.y = 0.1;
    wall2.primitive_poses[0].position.z = 0.03;
    wall2.primitive_poses[0].orientation.w = 1.0;
    wall2.operation = wall2.ADD;
    objects.push_back(wall2);

    // ---- Tubes -------------------------------------------------------------
    for (int i = 1; i <= NUM_TUBES; ++i)
    {
        double mx = 0.0, my = 0.0;
        if (!lookupMarkerXY(tf_buffer, frame_id, i, mx, my))
        {
            std::cout << "    [-] TF for marker_" << i << " not ready. Using fallback.\n";
            mx = TUBE_OBJ_FALLBACK_X[i - 1];
            my = TUBE_OBJ_FALLBACK_Y - TUBE_Y_OFFSET;
        }
        objects.push_back(makeTube(i, frame_id, mx, my));
    }

    // ---- Mixing Vessel -----------------------------------------------------
    double mixer_x = MIXER_OBJ_FALLBACK_X, mixer_y = MIXER_OBJ_FALLBACK_Y;
    if (!lookupMarkerXY(tf_buffer, frame_id, MIXER_MARKER, mixer_x, mixer_y))
        std::cout << "    [-] TF for marker_6 not ready. Using fallback mixer pose.\n";
    objects.push_back(makeMixer(frame_id, mixer_x, mixer_y));

    psi.applyCollisionObjects(objects);
    std::cout << ">>> [INIT] Collision objects loaded.\n";
}

static void waitForStateSettle(MGI & iface, int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0);
}

// Pour pose

static void mixerPourPose(double marker_x, double marker_y,
                          double & tcp_x, double & tcp_y, double & tcp_z)
{
    tcp_x = marker_x + POUR_X_OFFSET;
    tcp_y = marker_y + POUR_Y_OFFSET;
    tcp_z = POUR_Z;

    const double body_x = marker_x;
    const double body_y = marker_y + MIXER_Y_OFFSET;
    const double tube_x = tcp_x + 0.0;
    const double tube_y = tcp_y + TUBE_TCP_Y_OFFSET;
    const double tube_bottom = tcp_z + TUBE_TCP_Z_OFFSET - TUBE_HEIGHT / 2.0;

    const double dx = tube_x - body_x;
    const double dy = tube_y - body_y;
    const double gap = std::sqrt(dx * dx + dy * dy);

    std::cout << "    [mixer] body at (" << body_x << ", " << body_y
              << "), radius " << MIXER_RADIUS << ", rim z=" << MIXER_TOP_Z << "\n"
              << "    [mixer] TCP -> (" << tcp_x << ", " << tcp_y << ", " << tcp_z << ")\n"
              << "    [mixer] tube centre lands " << (gap * 1000.0)
              << " mm from the mixer axis; bottom at z=" << tube_bottom;

    if (tube_bottom < MIXER_TOP_Z && gap < MIXER_RADIUS)
        std::cout << "  <-- INSIDE the mixer, raise POUR_Z";
    else if (gap > MIXER_RADIUS + 0.12)
        std::cout << "  <-- far from the mixer, tilting may miss";
    std::cout << "\n";
}

//  MOTION HELPERS
// Cartesian interpolator
// Collision-aware IK.

class IkValidator
{
public:
    explicit IkValidator(const rclcpp::Node::SharedPtr & node,
                         const std::string & robot_description = "robot_description")
    {
        node_ = node;
        scene_pub_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(1));
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_description);
        psm_->startSceneMonitor("/monitored_planning_scene");
        psm_->startWorldGeometryMonitor();
        psm_->startStateMonitor();
        if (!psm_->requestPlanningSceneState("/get_planning_scene"))
            std::cout << "    [!] Could not fetch the initial planning scene; "
                         "IK collision checks may be unreliable at startup.\n";
    }

    moveit::core::GroupStateValidityCallbackFn callback()
    {
        auto psm = psm_;
        return [psm](moveit::core::RobotState * state,
                     const moveit::core::JointModelGroup * group,
                     const double * values) -> bool
        {
            state->setJointGroupPositions(group, values);
            state->update();
            planning_scene_monitor::LockedPlanningSceneRO scene(psm);
            if (!scene) return true;             // no scene yet: do not block IK
            return !scene->isStateColliding(*state, group->getName());
        };
    }

    const planning_scene_monitor::PlanningSceneMonitorPtr & psm() const { return psm_; }

    bool isStateValid(const moveit::core::RobotState & state, const std::string & group)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        if (!scene) return true;
        return !scene->isStateColliding(state, group);
    }

    // WHICH bodies are touching, rather than just that something is.

    std::vector<std::string> contactPairs(const moveit::core::RobotState & state,
                                          const std::string & group,
                                          std::size_t max_contacts = 20)
    {
        std::vector<std::string> out;
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        if (!scene) return out;

        collision_detection::CollisionRequest req;
        req.contacts     = true;
        req.max_contacts = max_contacts;
        req.max_contacts_per_pair = 1;
        req.group_name   = group;
        req.distance     = false;
        req.verbose      = false;

        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, state, scene->getAllowedCollisionMatrix());

        if (!res.collision) return out;

        for (const auto & pair : res.contacts)
        {
            std::ostringstream line;
            line << pair.first.first << "  <->  " << pair.first.second;
            if (!pair.second.empty())
                line << "   (depth " << (pair.second.front().depth * 1000.0) << " mm)";
            out.push_back(line.str());
        }
        if (out.empty())   // collision reported but no pairs captured
            out.push_back("<collision reported, but no contact pairs returned>");
        return out;
    }

    // Allow contact between an object and a set of links.
    void allowCollisions(const std::string & object,
                         const std::vector<std::string> & links,
                         bool allow)
    {
        moveit_msgs::msg::PlanningScene diff;
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
            if (!scene) return;
            collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
            acm.setEntry(object, links, allow);
            acm.getMessage(diff.allowed_collision_matrix);
        }
        diff.is_diff = true;
        scene_pub_->publish(diff);

        rclcpp::sleep_for(std::chrono::milliseconds(150));
        psm_->requestPlanningSceneState("/get_planning_scene");

        std::cout << "    [acm] " << (allow ? "allowing" : "forbidding")
                  << " contact between " << object << " and the gripper\n";
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

// Find a collision-free IK solution close to a seed configuration.

static bool solveNearestIk(MGI & mg,
                           IkValidator & validator,
                           const geometry_msgs::msg::Pose & pose,
                           const moveit::core::RobotState & seed_state,
                           moveit::core::RobotState & out_state,
                           int tries = 15,
                           double seed_spread = 0.8,
                           double ik_timeout = 0.05)
{
    const auto * jmg = seed_state.getJointModelGroup(mg.getName());
    if (!jmg) return false;

    const std::string tip = mg.getEndEffectorLink();
    auto validity = validator.callback();

    std::vector<double> q_seed;
    seed_state.copyJointGroupPositions(jmg, q_seed);

    double best_cost = std::numeric_limits<double>::max();
    bool found = false;

    for (int i = 0; i < tries; ++i)
    {
        moveit::core::RobotState s(seed_state);
        if (i > 0) s.setToRandomPositionsNearBy(jmg, seed_state, seed_spread);

        if (!s.setFromIK(jmg, pose, tip, ik_timeout, validity)) continue;
        if (!s.satisfiesBounds(jmg)) continue;

        std::vector<double> q;
        s.copyJointGroupPositions(jmg, q);

        double cost = 0.0;
        for (size_t k = 0; k < q.size(); ++k)
        {
            const double w = 1.0 + 0.5 * static_cast<double>(q.size() - k);
            cost += w * std::fabs(q[k] - q_seed[k]);
        }

        if (cost < best_cost) { best_cost = cost; out_state = s; found = true; }
    }

    return found;
}

// kinematic reachability test

static bool poseIsReachable(MGI & mg,
                            const geometry_msgs::msg::Pose & pose,
                            const moveit::core::RobotState & seed_a,
                            const moveit::core::RobotState & seed_b,
                            int tries = 6,
                            double ik_timeout = 0.02)
{
    const auto * jmg = seed_a.getJointModelGroup(mg.getName());
    if (!jmg) return true;
    const std::string tip = mg.getEndEffectorLink();

    for (int i = 0; i < tries; ++i)
    {
        const moveit::core::RobotState & base = (i % 2 == 0) ? seed_a : seed_b;
        moveit::core::RobotState s(base);
        if (i >= 2) s.setToRandomPositionsNearBy(jmg, base, 0.8);
        if (s.setFromIK(jmg, pose, tip, ik_timeout) && s.satisfiesBounds(jmg))
            return true;
    }
    return false;
}

static bool setNearestJointGoal(MGI & mg,
                                IkValidator & validator,
                                const geometry_msgs::msg::Pose & pose)
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    moveit::core::RobotState goal(*current);
    if (!solveNearestIk(mg, validator, pose, *current, goal)) return false;

    mg.setJointValueTarget(goal);
    return true;
}

// Time-parameterisation and execution of a raw trajectory

[[maybe_unused]] static bool retimeAndExecute(MGI & mg,
                             moveit_msgs::msg::RobotTrajectory & traj_msg,
                             double vel_scale,
                             double acc_scale)
{
    auto current_ptr = mg.getCurrentState(2.0);
    if (!current_ptr) return false;

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    rt.setRobotTrajectoryMsg(*current_ptr, traj_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale))
    {
        std::cout << "    [-] Time parameterisation failed.\n";
        return false;
    }
    rt.getRobotTrajectoryMsg(traj_msg);

    return mg.execute(traj_msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

//  INTERPOLATOR

struct CartesianOptions
{
    double step               = 0.005;   // nominal spacing between waypoints, m
    double max_joint_step     = 0.30;    // rad; reject a waypoint that jumps more
    int    seeds_per_waypoint = 5;       // perturbed-seed retries per waypoint
    int    max_subdivisions   = 4;       // how many times to halve a failing step
    double ik_timeout         = 0.02;

    // ---- Free-axis relaxation ---------------------------------------------
    double          free_axis_tolerance = 0.0;               
    Eigen::Vector3d free_axis           = Eigen::Vector3d::UnitX();  
    int             free_axis_samples   = 9;
};

// Straight-line blend for position, slerp for orientation.
static Eigen::Isometry3d lerpPose(const Eigen::Isometry3d & a,
                                  const Eigen::Isometry3d & b,
                                  double t)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = (1.0 - t) * a.translation() + t * b.translation();
    const Eigen::Quaterniond qa(a.rotation());
    const Eigen::Quaterniond qb(b.rotation());
    out.linear() = qa.slerp(t, qb).toRotationMatrix();
    return out;
}

static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & p)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    out.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                      p.orientation.y, p.orientation.z)
                       .normalized().toRotationMatrix();
    return out;
}

static double maxJointDelta(const moveit::core::RobotState & a,
                            const moveit::core::RobotState & b,
                            const moveit::core::JointModelGroup * jmg)
{
    std::vector<double> qa, qb;
    a.copyJointGroupPositions(jmg, qa);
    b.copyJointGroupPositions(jmg, qb);
    double worst = 0.0;
    for (size_t i = 0; i < qa.size(); ++i)
        worst = std::max(worst, std::fabs(qa[i] - qb[i]));
    return worst;
}

// Solve a single waypoint

static bool solveWaypoint(const moveit::core::JointModelGroup * jmg,
                          const std::string & tip,
                          IkValidator & validator,
                          const Eigen::Isometry3d & goal_pose,
                          const moveit::core::RobotState & prev,
                          const CartesianOptions & opts,
                          moveit::core::RobotState & out)
{
    auto validity = validator.callback();

    auto attempt = [&](const Eigen::Isometry3d & pose,
                       const moveit::core::RobotState & seed) -> bool
    {
        moveit::core::RobotState s(seed);
        if (!s.setFromIK(jmg, pose, tip, opts.ik_timeout, validity)) return false;
        if (!s.satisfiesBounds(jmg)) return false;
        if (maxJointDelta(prev, s, jmg) > opts.max_joint_step) return false;
        out = s;
        return true;
    };

    // 1. Exact pose, seeded from the previous solution.
    if (attempt(goal_pose, prev)) return true;

    // 2. Rotations about the free axis
    if (opts.free_axis_tolerance > 1e-6 && opts.free_axis_samples > 1)
    {
        const int half = std::max(1, opts.free_axis_samples / 2);
        for (int k = 1; k <= half; ++k)
        {
            const double mag = opts.free_axis_tolerance *
                               static_cast<double>(k) / static_cast<double>(half);
            for (const double sign : { 1.0, -1.0 })
            {
                Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
                r.linear() = Eigen::AngleAxisd(sign * mag, opts.free_axis.normalized())
                                 .toRotationMatrix();
                if (attempt(goal_pose * r, prev)) return true;
            }
        }
    }

    // 3. Perturbed seeds, exact pose.
    for (int i = 0; i < opts.seeds_per_waypoint; ++i)
    {
        moveit::core::RobotState seed(prev);
        seed.setToRandomPositionsNearBy(jmg, prev, 0.25);
        if (attempt(goal_pose, seed)) return true;
    }

    return false;
}

// Explain a waypoint that solveWaypoint() could not solve.
//
//   (a) no IK solution exists at that pose at all
//   (b) solutions exist but every one is in collision
//   (c) solutions exist and are collision-free, but sit further than
//       max_joint_step from the previous waypoint
//       continuity rejection, NOT an obstacle

static void diagnoseBlockedWaypoint(const moveit::core::JointModelGroup * jmg,
                                    const std::string & tip,
                                    IkValidator & validator,
                                    const Eigen::Isometry3d & goal_pose,
                                    const moveit::core::RobotState & prev,
                                    const CartesianOptions & opts,
                                    double t_blocked,
                                    double dt_smallest)
{
    const auto & jnames = jmg->getActiveJointModelNames();

    std::cout << "\n"
              << "    ==================== BLOCKED WAYPOINT ====================\n"
              << "    Stalled at t = " << t_blocked
              << " (smallest step tried: dt = " << dt_smallest << ")\n";

    const Eigen::Vector3d p = goal_pose.translation();
    std::cout << "    Target pose of the blocked waypoint: ("
              << p.x() << ", " << p.y() << ", " << p.z() << ")\n";

    // ---- Re-solve with NO validity callback ------------------------------

    std::vector<moveit::core::RobotState> raw;
    std::vector<double> raw_radius;
    const int probes = 40;
    for (int i = 0; i < probes; ++i)
    {
        const double radius = (i < 20) ? 0.25 : 0.8;
        moveit::core::RobotState s(prev);
        if (i > 0) s.setToRandomPositionsNearBy(jmg, prev, radius);
        if (s.setFromIK(jmg, goal_pose, tip, opts.ik_timeout))
        {
            s.update();
            raw.push_back(s);
            raw_radius.push_back(i == 0 ? 0.0 : radius);
        }
    }

    if (raw.empty())
    {
        std::cout << "    (a) NO IK SOLUTION at this pose, collisions aside.\n"
                     "        " << probes << " seeded attempts, none converged. This is a\n"
                     "        reach / wrist-singularity problem. Collision objects are\n"
                     "        NOT involved -- do not go looking for them.\n"
                  << "    ==========================================================\n\n";
        return;
    }

    // ---- Grade every raw solution against the three gates ------------------

    int n_out_of_bounds = 0, n_colliding = 0, n_too_far = 0, n_would_pass = 0;
    double best_delta = std::numeric_limits<double>::infinity();
    std::string best_delta_joint = "?";
    std::vector<std::string> first_contacts;
    std::vector<double> best_q;

    int n_pass_narrow = 0, n_pass_wide = 0;

    for (size_t idx = 0; idx < raw.size(); ++idx)
    {
        const auto & s = raw[idx];
        if (!s.satisfiesBounds(jmg)) { ++n_out_of_bounds; continue; }

        const bool free_of_collision = validator.isStateValid(s, jmg->getName());
        if (!free_of_collision)
        {
            ++n_colliding;
            if (first_contacts.empty())
                first_contacts = validator.contactPairs(s, jmg->getName());
            continue;
        }

        std::vector<double> qa, qb;
        prev.copyJointGroupPositions(jmg, qa);
        s.copyJointGroupPositions(jmg, qb);
        double worst = 0.0;
        std::string worst_joint = "?";
        for (size_t j = 0; j < qa.size() && j < jnames.size(); ++j)
        {
            const double d = std::fabs(qa[j] - qb[j]);
            if (d > worst) { worst = d; worst_joint = jnames[j]; }
        }
        if (worst < best_delta)
        {
            best_delta = worst;
            best_delta_joint = worst_joint;
            best_q = qb;
        }

        if (worst > opts.max_joint_step)
        {
            ++n_too_far;
        }
        else
        {
            ++n_would_pass;
            if (raw_radius[idx] <= 0.3) ++n_pass_narrow;
            else                        ++n_pass_wide;
        }
    }

    std::cout << "    " << raw.size() << " raw IK solution(s) found with the collision\n"
              << "    check disabled. Grading them:\n"
              << "        out of joint bounds .......... " << n_out_of_bounds << "\n"
              << "        in collision ................. " << n_colliding << "\n"
              << "        collision-free but too far ... " << n_too_far
              << "  (limit " << opts.max_joint_step << " rad)\n"
              << "        should have passed ........... " << n_would_pass << "\n";

    if (n_colliding > 0)
    {
        std::cout << "\n    (b) CONTACT PAIRS at the first colliding solution:\n";
        if (first_contacts.empty())
            std::cout << "        <none captured>\n";
        for (const auto & c : first_contacts)
            std::cout << "        " << c << "\n";
    }

    if (std::isfinite(best_delta))
    {
        std::cout << "        closest collision-free solution: " << best_delta
                  << " rad on '" << best_delta_joint << "'\n";

        // Side-by-side joint values with their limits
        std::vector<double> q_prev;
        prev.copyJointGroupPositions(jmg, q_prev);
        std::cout << "\n        joint      last good (t=" << (t_blocked - dt_smallest)
                  << ")   needed here   delta      limits\n";
        for (size_t j = 0; j < q_prev.size() && j < best_q.size() && j < jnames.size(); ++j)
        {
            const auto * jm = jmg->getActiveJointModels()[j];
            std::string lim = "(none)";
            if (jm && !jm->getVariableBoundsMsg().empty())
            {
                const auto & b = jm->getVariableBoundsMsg().front();
                std::ostringstream ls;
                ls << "[" << b.min_position << ", " << b.max_position << "]";
                lim = ls.str();
            }
            std::cout << "        " << jnames[j] << "   " << q_prev[j]
                      << "     " << best_q[j]
                      << "     " << (best_q[j] - q_prev[j])
                      << "     " << lim << "\n";
        }
    }

    // The verdict
    if (n_would_pass == 0 && n_too_far > 0)
    {
        std::cout << "\n    (c) CONTINUITY REJECTION, not a collision. EVERY reachable,\n"
                     "        collision-free solution is a null-space jump away from the\n"
                     "        previous waypoint -- closest is " << best_delta
                  << " rad on joint\n        '" << best_delta_joint
                  << "' vs a budget of " << opts.max_joint_step << " rad. Widening\n"
                     "        max_joint_step is the only thing that can help here.\n";
    }
    else if (n_would_pass > 0)
    {
        std::cout << "\n    (d) SEEDING failure, not geometry. " << n_would_pass
                  << " solution(s) passed all\n        three gates, yet solveWaypoint()'s "
                  << (1 + opts.seeds_per_waypoint) << " attempts did not find one.\n"
                     "        Found from the narrow (0.25 rad) seed radius solveWaypoint\n"
                     "        actually uses: " << n_pass_narrow
                  << ";  from the wider 0.8 radius: " << n_pass_wide << ".\n";
        if (n_pass_narrow == 0)
            std::cout << "        -> NONE from the narrow radius: solveWaypoint's seed spread\n"
                         "           is too tight to reach them at all.\n";
        else
            std::cout << "        -> Reachable from the narrow radius: more attempts or a\n"
                         "           longer ik_timeout (now " << opts.ik_timeout
                      << " s) would find them.\n";
    }

    std::cout << "    ==========================================================\n\n";
}

// The interpolator itself. Returns the fraction of the line achieved and fills
// out with whatever it managed to build.

static double planRobustCartesian(MGI & mg,
                                  IkValidator & validator,
                                  const geometry_msgs::msg::Pose & target,
                                  const CartesianOptions & opts,
                                  robot_trajectory::RobotTrajectory & out,
                                  const moveit::core::RobotState * start_state = nullptr,
                                  bool verbose = true)
{
    moveit::core::RobotStatePtr current;
    if (start_state)
        current = std::make_shared<moveit::core::RobotState>(*start_state);
    else
        current = mg.getCurrentState(2.0);
    if (!current) return 0.0;

    const auto * jmg = current->getJointModelGroup(mg.getName());
    if (!jmg) return 0.0;
    const std::string tip = mg.getEndEffectorLink();

    const Eigen::Isometry3d start_pose = current->getGlobalLinkTransform(tip);
    const Eigen::Isometry3d end_pose   = poseMsgToEigen(target);

    const double distance = (end_pose.translation() - start_pose.translation()).norm();
    if (distance < 1e-6) return 1.0;

    const double dt_nominal = std::min(1.0, opts.step / distance);

    out.clear();
    out.addSuffixWayPoint(*current, 0.0);

    moveit::core::RobotState prev(*current);
    double t_done = 0.0;
    int refinements = 0;
    int guard = 0;

    while (t_done < 1.0 - 1e-9 && guard++ < 4000)
    {
        double dt = dt_nominal;
        bool advanced = false;
        double dt_last = dt;

        for (int sub = 0; sub <= opts.max_subdivisions; ++sub)
        {
            const double t_next = std::min(1.0, t_done + dt);
            const Eigen::Isometry3d goal_i = lerpPose(start_pose, end_pose, t_next);

            moveit::core::RobotState s(prev);
            if (solveWaypoint(jmg, tip, validator, goal_i, prev, opts, s))
            {
                out.addSuffixWayPoint(s, 0.0);
                prev = s;
                t_done = t_next;
                advanced = true;
                if (sub > 0) ++refinements;
                break;
            }

            dt_last = dt;
            dt *= 0.5;
        }

        if (!advanced)
        {
            if (verbose)
            {
                const double t_blocked = std::min(1.0, t_done + dt_last);
                diagnoseBlockedWaypoint(jmg, tip, validator,
                                        lerpPose(start_pose, end_pose, t_blocked),
                                        prev, opts, t_blocked, dt_last);
            }
            break;
        }
    }

    if (verbose && refinements > 0)
        std::cout << "    [cart] refined the step " << refinements
                  << " time(s) to get through tight spots.\n";

    return t_done;
}

// Reports the outcome of every command back to whoever sent it.

struct CommandReport
{
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    std::string cmd;
    bool ok{ false };

    ~CommandReport()
    {
        if (!pub) return;
        std_msgs::msg::String msg;
        msg.data = (ok ? "OK " : "FAIL ") + cmd;
        pub->publish(msg);
        std::cout << "    [status] " << msg.data << "\n";
    }
};

// Elapsed-time helper for the approach stage breakdown

struct Stopwatch
{
    std::chrono::steady_clock::time_point t0{ std::chrono::steady_clock::now() };
    double ms()
    {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0).count();
    }
    double lap()
    {
        const double v = ms();
        t0 = std::chrono::steady_clock::now();
        return v;
    }
};

// Smallest distance from any active joint to its own nearest limit.

static double minLimitHeadroom(const moveit::core::RobotState & state,
                               const moveit::core::JointModelGroup * jmg,
                               std::string * worst_joint = nullptr)
{
    std::vector<double> q;
    state.copyJointGroupPositions(jmg, q);
    const auto & jnames = jmg->getActiveJointModelNames();
    const auto & jmodels = jmg->getActiveJointModels();

    double worst = std::numeric_limits<double>::max();
    for (size_t j = 0; j < q.size() && j < jmodels.size(); ++j)
    {
        const auto * jm = jmodels[j];
        if (!jm || jm->getVariableBoundsMsg().empty()) continue;
        const auto & b = jm->getVariableBoundsMsg().front();
        if (!b.has_position_limits) continue;

        const double head = std::min(q[j] - b.min_position, b.max_position - q[j]);
        if (head < worst)
        {
            worst = head;
            if (worst_joint && j < jnames.size()) *worst_joint = jnames[j];
        }
    }
    return (worst == std::numeric_limits<double>::max()) ? 0.0 : worst;
}

// The same wrist orientation, expressed on the other ZYZ branch.

static moveit::core::RobotState mirrorWristSeed(const moveit::core::RobotState & s,
                                                const moveit::core::JointModelGroup * jmg)
{
    moveit::core::RobotState out(s);
    std::vector<double> q;
    out.copyJointGroupPositions(jmg, q);
    if (q.size() >= 7)
    {
        q[4] += (q[4] > 0.0) ? -M_PI : M_PI;
        q[5]  = -q[5];
        q[6] += (q[6] > 0.0) ? -M_PI : M_PI;
        out.setJointGroupPositions(jmg, q);
        out.enforceBounds(jmg);
        out.update();
    }
    return out;
}

static double descentDryRun(MGI & mg,
                            IkValidator & validator,
                            const moveit::core::RobotState & from,
                            const geometry_msgs::msg::Pose & target,
                            double free_axis_tolerance)
{
    CartesianOptions opts;
    opts.free_axis_tolerance = 0.0;

    robot_trajectory::RobotTrajectory traj(mg.getRobotModel(), mg.getName());
    double f = planRobustCartesian(mg, validator, target, opts, traj, &from, false);

    if (f < 0.995 && free_axis_tolerance > 1e-6)
    {
        opts.free_axis_tolerance = free_axis_tolerance;
        robot_trajectory::RobotTrajectory relaxed(mg.getRobotModel(), mg.getName());
        f = std::max(f, planRobustCartesian(mg, validator, target, opts, relaxed,
                                            &from, false));
    }
    return f;
}

// Pure joint-space interpolation between the current state and a target joint
// configuration. Calls IK exactly ONCE instead of once per waypoint, so it either works or fails

static bool moveJointInterpolated(MGI & mg,
                                  IkValidator & validator,
                                  const moveit::core::RobotState & goal_state,
                                  double vel_scale,
                                  double acc_scale,
                                  int steps = 30,
                                  double max_deviation = 0.006,
                                  const geometry_msgs::msg::Quaternion * tilt_reference = nullptr,
                                  double max_tilt = 0.0)
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    const auto * jmg = current->getJointModelGroup(mg.getName());
    const std::string tip = mg.getEndEffectorLink();

    const Eigen::Vector3d p_start = current->getGlobalLinkTransform(tip).translation();
    const Eigen::Vector3d p_end   = goal_state.getGlobalLinkTransform(tip).translation();
    const Eigen::Vector3d line    = p_end - p_start;
    const double line_len = line.norm();

    Eigen::Vector3d ref_axis = Eigen::Vector3d::UnitZ();
    if (tilt_reference)
    {
        const Eigen::Quaterniond q_ref(tilt_reference->w, tilt_reference->x,
                                       tilt_reference->y, tilt_reference->z);
        ref_axis = (q_ref.normalized() * Eigen::Vector3d::UnitX()).normalized();
    }

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    double worst_dev = 0.0;
    double worst_tilt = 0.0;

    for (int i = 0; i <= steps; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        moveit::core::RobotState s(*current);
        current->interpolate(goal_state, t, s, jmg);
        s.update();

        if (!validator.isStateValid(s, mg.getName()))
        {
            std::cout << "    [-] Joint interpolation hits a collision at t=" << t << ".\n";
            return false;
        }

        if (tilt_reference)
        {
            const Eigen::Vector3d a =
                (s.getGlobalLinkTransform(tip).linear() * Eigen::Vector3d::UnitX()).normalized();
            const double c = std::max(-1.0, std::min(1.0, a.dot(ref_axis)));
            worst_tilt = std::max(worst_tilt, std::acos(c));
            if (worst_tilt > max_tilt)
            {
                std::cout << "    [-] Joint interpolation leans the tool "
                          << (worst_tilt * 180.0 / M_PI) << " deg at t=" << t
                          << "; refusing (limit " << (max_tilt * 180.0 / M_PI)
                          << " deg).\n";
                return false;
            }
        }

        if (line_len > 1e-6)
        {
            const Eigen::Vector3d p = s.getGlobalLinkTransform(tip).translation();
            const Eigen::Vector3d v = p - p_start;
            const double proj = v.dot(line) / line_len;
            const double dev = (v - (proj / line_len) * line).norm();
            worst_dev = std::max(worst_dev, dev);
        }

        rt.addSuffixWayPoint(s, 0.0);
    }

    std::cout << "    [joint] max deviation from the straight line = "
              << (worst_dev * 1000.0) << " mm\n";

    if (worst_dev > max_deviation)
    {
        std::cout << "    [-] Deviation exceeds " << (max_deviation * 1000.0)
                  << " mm; refusing.\n";
        return false;
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale)) return false;

    moveit_msgs::msg::RobotTrajectory msg;
    rt.getRobotTrajectoryMsg(msg);
    return mg.execute(msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

// Straight-line move with a three-rung fallback ladder:
//   1. interpolator, exact orientation
//   2. interpolator, allowing rotation about the tool axis
//   3. joint interpolation between IK-verified endpoints

static bool moveLinear(MGI & mg,
                       IkValidator & validator,
                       const geometry_msgs::msg::Pose & target,
                       double vel_scale,
                       double acc_scale,
                       double free_axis_tolerance = 0.0,
                       double min_fraction = 0.995)
{
    CartesianOptions opts;
    opts.free_axis_tolerance = 0.0;

    robot_trajectory::RobotTrajectory traj(mg.getRobotModel(), mg.getName());

    double fraction = planRobustCartesian(mg, validator, target, opts, traj);
    std::cout << "    [cart] strict: " << (fraction * 100.0) << "%\n";

    if (fraction < min_fraction && free_axis_tolerance > 1e-6)
    {
        opts.free_axis_tolerance = free_axis_tolerance;
        robot_trajectory::RobotTrajectory relaxed(mg.getRobotModel(), mg.getName());
        const double f2 = planRobustCartesian(mg, validator, target, opts, relaxed);
        std::cout << "    [cart] free-axis relaxed: " << (f2 * 100.0) << "%\n";
        if (f2 > fraction) { fraction = f2; traj = relaxed; }
    }

    if (fraction >= min_fraction)
    {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(traj, vel_scale, acc_scale))
        {
            std::cout << "    [-] Time parameterisation failed.\n";
            return false;
        }
        moveit_msgs::msg::RobotTrajectory msg;
        traj.getRobotTrajectoryMsg(msg);
        return mg.execute(msg) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    std::cout << "    [*] Straight-line planning stalled; trying joint interpolation.\n";

    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    moveit::core::RobotState goal(*current);
    if (!solveNearestIk(mg, validator, target, *current, goal))
    {
        std::cout << "    [-] No collision-free IK at the target at all. The pose is\n"
                     "        unreachable, not merely hard to reach in a straight line.\n";
        return false;
    }

    return moveJointInterpolated(mg, validator, goal, vel_scale, acc_scale);
}

// Free-space move with obstacle avoidance, via a validated joint goal.

static bool moveFreeSpace(MGI & mg,
                          IkValidator & validator,
                          const geometry_msgs::msg::Pose & target,
                          double vel_scale,
                          double acc_scale)
{
    mg.setStartStateToCurrentState();
    mg.clearPoseTargets();
    mg.setMaxVelocityScalingFactor(vel_scale);
    mg.setMaxAccelerationScalingFactor(acc_scale);

    if (!setNearestJointGoal(mg, validator, target))
    {
        std::cout << "    [!] No collision-free IK; falling back to a pose goal.\n";
        mg.setPoseTarget(target);
    }

    MGI::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        return false;

    return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

static moveit_msgs::msg::Constraints uprightConstraint(
    const std::string & tcp_link,
    const std::string & planning_frame,
    const geometry_msgs::msg::Quaternion & upright,
    double tilt_tolerance = TILT_TOLERANCE)
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name                 = tcp_link;
    ocm.header.frame_id           = planning_frame;
    ocm.orientation               = upright;
    ocm.absolute_x_axis_tolerance = M_PI;             // roll about the tube -- free
    ocm.absolute_y_axis_tolerance = tilt_tolerance;   // tipping -- keep tight
    ocm.absolute_z_axis_tolerance = tilt_tolerance;   // tipping -- keep tight
    ocm.weight                    = 1.0;

    ocm.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;

    moveit_msgs::msg::Constraints c;
    c.orientation_constraints.push_back(ocm);
    return c;
}

// Orientation path constraint for carrying a tube.

static const moveit_msgs::msg::RobotTrajectory & planTraj(const MGI::Plan & p)
{
#ifdef MOVEIT_JAZZY_OR_NEWER
    return p.trajectory;
#else
    return p.trajectory_;
#endif
}

// Worst tilt of the tool axis away from its reference direction

static double maxTiltAlongTrajectory(MGI & mg,
                                     const moveit_msgs::msg::RobotTrajectory & traj_msg,
                                     const geometry_msgs::msg::Quaternion & reference,
                                     const Eigen::Vector3d & tool_axis = Eigen::Vector3d::UnitX())
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return 0.0;

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    rt.setRobotTrajectoryMsg(*current, traj_msg);

    const Eigen::Quaterniond q_ref(reference.w, reference.x, reference.y, reference.z);
    const Eigen::Vector3d ref_axis = (q_ref.normalized() * tool_axis).normalized();

    const std::string tip = mg.getEndEffectorLink();
    double worst = 0.0;

    for (size_t i = 0; i < rt.getWayPointCount(); ++i)
    {
        const Eigen::Vector3d a =
            (rt.getWayPoint(i).getGlobalLinkTransform(tip).linear() * tool_axis).normalized();
        const double c = std::max(-1.0, std::min(1.0, a.dot(ref_axis)));
        worst = std::max(worst, std::acos(c));
    }
    return worst;
}

// Free-space move that keeps the tool upright, with a three-rung ladder.
//   1. OMPL with the tight constraint
//   2. OMPL with a loosened constraint
//   3. OMPL with NO constraint, then verify the resulting trajectory and reject
//      it if the tube would ever tip past hard_limit

static bool moveFreeSpaceUpright(MGI & mg,
                                 IkValidator & validator,
                                 const geometry_msgs::msg::Pose & target,
                                 double vel_scale,
                                 double acc_scale,
                                 const std::string & tcp_link,
                                 const std::string & planning_frame,
                                 double tight_tol = TILT_TOLERANCE,
                                 double loose_tol = 0.5,
                                 double hard_limit = 0.7)
{
    auto try_plan = [&](const moveit_msgs::msg::Constraints * c,
                        const char * label) -> bool
    {
        mg.setStartStateToCurrentState();
        mg.clearPoseTargets();
        mg.setMaxVelocityScalingFactor(vel_scale);
        mg.setMaxAccelerationScalingFactor(acc_scale);

        if (c) mg.setPathConstraints(*c);
        else   mg.clearPathConstraints();

        if (!setNearestJointGoal(mg, validator, target))
        {
            std::cout << "    [" << label << "] no collision-free IK at the goal.\n";
            return false;
        }

        MGI::Plan plan;
        if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            std::cout << "    [" << label << "] planning failed.\n";
            return false;
        }

        const double tilt = maxTiltAlongTrajectory(mg, planTraj(plan), target.orientation);
        std::cout << "    [" << label << "] ok, max tilt "
                  << (tilt * 180.0 / M_PI) << " deg\n";

        if (tilt > hard_limit)
        {
            std::cout << "    [" << label << "] rejected: exceeds the hard tilt limit of "
                      << (hard_limit * 180.0 / M_PI) << " deg.\n";
            return false;
        }

        return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    };

    const auto tight = uprightConstraint(tcp_link, planning_frame, target.orientation, tight_tol);
    if (try_plan(&tight, "upright/tight")) { mg.clearPathConstraints(); return true; }

    const auto loose = uprightConstraint(tcp_link, planning_frame, target.orientation, loose_tol);
    if (try_plan(&loose, "upright/loose")) { mg.clearPathConstraints(); return true; }

    std::cout << "    [*] Constrained planning failed both times; planning freely\n"
                 "        and checking the result instead.\n";
    const bool ok = try_plan(nullptr, "upright/verified");
    mg.clearPathConstraints();
    return ok;
}

// 5-DOF UPRIGHT MANIFOLD

static std::unique_ptr<my_robot_control::ManifoldPlanner> g_manifold;

static bool moveFreeSpaceManifold(MGI & mg,
                                  IkValidator & validator,
                                  const geometry_msgs::msg::Pose & target,
                                  double vel_scale,
                                  double acc_scale,
                                  double hard_limit = 0.7,
                                  const moveit::core::RobotState * goal_hint = nullptr)
{
    using my_robot_control::ManifoldPlanner;

    if (!g_manifold)
        return false;

    const auto current = mg.getCurrentState(2.0);
    if (!current)
    {
        std::cout << "    [manifold] no current state.\n";
        return false;
    }

    ManifoldPlanner::JointVector start;
    for (int k = 0; k < 7; ++k)
        start[k] = current->getVariablePosition("joint" + std::to_string(k + 1));

    const Eigen::Quaterniond q_goal(target.orientation.w, target.orientation.x,
                                    target.orientation.y, target.orientation.z);
    double roll = 0.0;
    if (!ManifoldPlanner::rollOfOrientation(q_goal.toRotationMatrix(), roll))
    {
        std::cout << "    [manifold] target is not upright; this planner cannot "
                     "represent it.\n";
        return false;
    }

    planning_scene::PlanningScenePtr scene;
    {
        planning_scene_monitor::LockedPlanningSceneRO ro(validator.psm());
        if (!ro)
        {
            std::cout << "    [manifold] no planning scene.\n";
            return false;
        }
        scene = planning_scene::PlanningScene::clone(ro);
    }
    g_manifold->setScene(scene, scene->getCurrentState());

    ManifoldPlanner::JointVector hint;
    const ManifoldPlanner::JointVector * hint_ptr = nullptr;
    if (goal_hint)
    {
        for (int k = 0; k < 7; ++k)
            hint[k] = goal_hint->getVariablePosition("joint" + std::to_string(k + 1));
        hint_ptr = &hint;
    }

    const Eigen::Vector3d tcp(target.position.x, target.position.y, target.position.z);
    const auto res = g_manifold->planToToolPose(start, tcp, &roll, hint_ptr);

    if (!res.success)
    {
        std::cout << "    [manifold] " << res.message << " ("
                  << res.goal_states << " goal states, "
                  << res.planning_time * 1e3 << " ms)\n";
        return false;
    }

    if (res.max_tilt > hard_limit)
    {
        std::cout << "    [manifold] rejected: tilt " << (res.max_tilt * 180.0 / M_PI)
                  << " deg exceeds the hard limit.\n";
        return false;
    }

    // ---- time parameterisation -----------------------------------------

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    moveit::core::RobotState wp(scene->getCurrentState());

    auto set_arm = [&wp](const ManifoldPlanner::JointVector & q)
    {
        for (int k = 0; k < 7; ++k)
        {
            const std::string n = "joint" + std::to_string(k + 1);
            const double v = q[k];
            wp.setJointPositions(n, &v);
        }
        wp.update();
    };

    const double bridge = (start - res.path.front()).cwiseAbs().maxCoeff();
    if (bridge > 1e-6)
    {

        constexpr double kMaxBridge = 0.25;
        if (bridge > kMaxBridge)
        {
            std::cout << "    [manifold] start is " << bridge
                      << " rad off the planned start (branch " << res.branch_used
                      << "); too far to bridge. Per joint:";
            for (int k = 0; k < 7; ++k)
                std::cout << " j" << (k + 1) << "=" << (res.path.front()[k] - start[k]);
            std::cout << "\n    [manifold] why: " << res.message << "\n";
            return false;
        }

        const int steps = std::max(2, static_cast<int>(std::ceil(bridge / 0.02)));
        for (int i = 0; i < steps; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const ManifoldPlanner::JointVector q =
                start + t * (res.path.front() - start);
            set_arm(q);
            if (!validator.isStateValid(wp, mg.getName()))
            {
                std::cout << "    [manifold] the approach onto the manifold collides.\n";
                return false;
            }
            rt.addSuffixWayPoint(wp, 0.0);
        }
        std::cout << "    [manifold] bridged " << bridge
                  << " rad from the current state onto the manifold.\n";
    }

    for (const auto & q : res.path)
    {
        set_arm(q);
        rt.addSuffixWayPoint(wp, 0.0);
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale))
    {
        std::cout << "    [manifold] time parameterisation failed.\n";
        return false;
    }

    moveit_msgs::msg::RobotTrajectory traj_msg;
    rt.getRobotTrajectoryMsg(traj_msg);

    std::cout << "    [manifold] ok: " << res.path.size() << " waypoints, "
              << res.goal_states << " goal states, planned in "
              << res.planning_time * 1e3 << " ms, tilt "
              << (res.max_tilt * 180.0 / M_PI) << " deg, max joint step "
              << res.max_joint_step << " rad\n";

    if (!res.path.empty())
        std::cout << "    [manifold] planned endpoint limit headroom: "
                  << g_manifold->limitHeadroom(res.path.back()) << " rad\n";

    return mg.execute(traj_msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

// Transit dispatch.
static bool g_use_manifold = true;
static int  g_manifold_ok = 0;
static int  g_manifold_fallback = 0;

// `free_fallback` allows a final unconstrained attempt when everything else fails

static bool transitUpright(MGI & mg,
                           IkValidator & validator,
                           const geometry_msgs::msg::Pose & target,
                           double vel_scale,
                           double acc_scale,
                           const std::string & tcp_link,
                           const std::string & planning_frame,
                           double tight_tol = TILT_TOLERANCE,
                           double loose_tol = 0.5,
                           double hard_limit = 0.7,
                           bool free_fallback = false,
                           const moveit::core::RobotState * goal_hint = nullptr)
{
    if (g_use_manifold &&
        moveFreeSpaceManifold(mg, validator, target, vel_scale, acc_scale, hard_limit,
                              goal_hint))
    {
        ++g_manifold_ok;
        return true;
    }

    if (g_use_manifold)
    {
        ++g_manifold_fallback;
        std::cout << "    [transit] falling back to constrained 7-DOF planning.\n";
    }

    if (moveFreeSpaceUpright(mg, validator, target, vel_scale, acc_scale,
                             tcp_link, planning_frame, tight_tol, loose_tol,
                             hard_limit))
        return true;

    if (free_fallback)
    {
        std::cout << "    [transit] empty gripper: dropping the orientation "
                     "requirement for a last attempt.\n";
        return moveFreeSpace(mg, validator, target, vel_scale, acc_scale);
    }
    return false;
}

// Retry wrapper.

template <typename F>
static bool withRetries(const char * what, int attempts, F && fn)
{
    for (int i = 1; i <= attempts; ++i)
    {
        if (fn())
        {
            if (i > 1)
                std::cout << "    [+] " << what << " succeeded on attempt " << i << ".\n";
            return true;
        }
        if (i < attempts)
        {
            std::cout << "    [retry] " << what << " failed (attempt " << i
                      << " of " << attempts << "); re-seeding...\n";
            rclcpp::sleep_for(std::chrono::milliseconds(400));
        }
    }
    std::cout << "    [-] " << what << " failed all " << attempts << " attempts.\n";
    return false;
}

// Backward-planned approach:
//   1. find a collision-free IK solution at the GRASP pose
//   2. seed the standoff IK from it, so both ends share an IK branch and the
//      arm cannot arrive at a standoff posture that runs out of joint range
//      part-way down
//   3. free-space transit to that standoff configuration
//   4. straight-line descent with the fallback ladder

static bool approachTarget(MGI & mg,
                           IkValidator & validator,
                           const geometry_msgs::msg::Pose & target,
                           double standoff_z,
                           double vel_scale,
                           double acc_scale,
                           double free_axis_tolerance = 0.0,
                           bool carrying = false,
                           const std::string & tcp_link = "",
                           const std::string & planning_frame = "")
{
    auto current = mg.getCurrentState(2.0);
    if (!current)
    {
        std::cout << "    [-] Could not read robot state.\n";
        return false;
    }

    std::cout << "    [target] (" << target.position.x << ", " << target.position.y
              << ", " << target.position.z << ")\n";

    Stopwatch sw_stage;   
    Stopwatch sw_total;   

    // ---- Step 1: is the FINAL pose reachable and collision-free? -----------

    moveit::core::RobotState grasp_state(*current);
    if (!solveNearestIk(mg, validator, target, *current, grasp_state))
    {
        const auto * jmg = current->getJointModelGroup(mg.getName());
        const std::string tip = mg.getEndEffectorLink();
        bool kinematically_reachable = false;
        for (int i = 0; i < 12 && !kinematically_reachable; ++i)
        {
            moveit::core::RobotState probe(*current);
            if (i > 0) probe.setToRandomPositionsNearBy(jmg, *current, 0.8);
            if (probe.setFromIK(jmg, target, tip, 0.05) && probe.satisfiesBounds(jmg))
                kinematically_reachable = true;
        }

        if (kinematically_reachable)
            std::cout << "    [-] IK exists at the target, but every solution is in\n"
                         "        COLLISION. Something is in the way -- check the\n"
                         "        collision objects around that pose in RViz.\n";
        else
            std::cout << "    [-] NO kinematic solution at the target at all, collisions\n"
                         "        aside. The pose is OUT OF REACH or the commanded\n"
                         "        orientation is impossible there. Collision objects are\n"
                         "        not the problem; the target itself is.\n";
        return false;
    }

    // ---- Step 2: find a standoff height that also has IK --------------------
    geometry_msgs::msg::Pose standoff = target;
    moveit::core::RobotState standoff_state(grasp_state);
    double used_standoff = -1.0;

    std::cout << "    [timing] grasp IK: " << sw_stage.lap() << " ms\n";

    const auto * arm_jmg = current->getJointModelGroup(mg.getName());

    moveit::core::RobotState any_unverified(grasp_state);
    geometry_msgs::msg::Pose any_unverified_pose = target;
    double any_unverified_z = 0.0;
    bool have_any_unverified = false;

    for (const double scale : { 1.0, 0.7, 0.45, 0.25, 0.0 })
    {
        const double sz = standoff_z * scale;
        if (sz < 0.005)
        {
            used_standoff = 0.0;
            break;
        }

        geometry_msgs::msg::Pose cand = target;
        cand.position.z += sz;

        {
            Stopwatch sw_reach;
            if (!poseIsReachable(mg, cand, grasp_state, *current))
            {
                std::cout << "    [standoff] z+" << sz << " is out of reach ("
                          << sw_reach.ms() << " ms); trying lower.\n";
                continue;
            }
        }

        const moveit::core::RobotState grasp_mirror = mirrorWristSeed(grasp_state, arm_jmg);
        const moveit::core::RobotState current_mirror = mirrorWristSeed(*current, arm_jmg);

        struct Candidate { const char * how; const moveit::core::RobotState * seed;
                           int tries; double spread; };
        const Candidate gens[] = {
            { "seeded from the grasp configuration", &grasp_state,    8, 0.30 },
            { "seeded from the current pose",        current.get(),  12, 0.80 },
            { "mirrored wrist, from the grasp config", &grasp_mirror, 12, 0.30 },
            { "mirrored wrist, from the current pose", &current_mirror, 12, 0.80 },
            { "wide re-seed from the grasp config",  &grasp_state,   16, 1.20 },
            { "wide mirrored re-seed",               &grasp_mirror,  16, 1.20 },
        };

        moveit::core::RobotState best_unverified(grasp_state);
        bool have_unverified = false;
        bool accepted = false;

        std::vector<moveit::core::RobotState> already_tried;

        for (const auto & gen : gens)
        {
            Stopwatch sw_gen;
            moveit::core::RobotState trial(grasp_state);
            if (!solveNearestIk(mg, validator, cand, *gen.seed, trial,
                                gen.tries, gen.spread))
            {
                std::cout << "    [standoff] no IK (" << gen.how << "), "
                          << sw_gen.ms() << " ms\n";
                continue;
            }
            const double ik_ms = sw_gen.lap();

            bool duplicate = false;
            for (const auto & seen : already_tried)
                if (maxJointDelta(trial, seen, arm_jmg) < 0.02) { duplicate = true; break; }
            if (duplicate)
            {
                std::cout << "    [standoff] skipped (" << gen.how
                          << "): same configuration as one already tested. [ik "
                          << ik_ms << " ms]\n";
                continue;
            }
            already_tried.push_back(trial);

            std::string tight_joint = "?";
            const double head = minLimitHeadroom(trial, arm_jmg, &tight_joint);

            if (!have_unverified) { best_unverified = trial; have_unverified = true; }

            if (head < LIMIT_HEADROOM_MIN)
            {
                std::cout << "    [standoff] rejected (" << gen.how << "): joint '"
                          << tight_joint << "' only " << head
                          << " rad from its limit. [ik " << ik_ms << " ms]\n";
                continue;
            }

            const double f = descentDryRun(mg, validator, trial, target,
                                           free_axis_tolerance);
            const double dry_ms = sw_gen.ms();
            if (f < 0.995)
            {
                std::cout << "    [standoff] rejected (" << gen.how
                          << "): dry-run descent reaches only " << (f * 100.0)
                          << "%. [ik " << ik_ms << " ms, dry-run " << dry_ms << " ms]\n";
                continue;
            }

            std::cout << "    [standoff] accepted (" << gen.how
                      << "): headroom " << head
                      << " rad, dry-run descent 100%. [ik " << ik_ms
                      << " ms, dry-run " << dry_ms << " ms]\n";
            standoff_state = trial;
            accepted = true;
            break;
        }

        if (accepted)
        {
            standoff = cand;
            used_standoff = sz;
            break;
        }

        if (have_unverified && !have_any_unverified)
        {
            any_unverified = best_unverified;
            any_unverified_pose = cand;
            any_unverified_z = sz;
            have_any_unverified = true;
        }
        std::cout << "    [!] No descendable standoff at z+" << sz
                  << "; trying lower.\n";
    }

    if (used_standoff < 0.0 && have_any_unverified)
    {
        std::cout << "    [!] No standoff height offered a verified descent; "
                     "falling back to the best configuration found.\n";
        standoff_state = any_unverified;
        standoff = any_unverified_pose;
        used_standoff = any_unverified_z;
    }

    if (used_standoff < 0.0)
    {
        std::cout << "    [-] No collision-free IK at any standoff height.\n";
        return false;
    }

    if (used_standoff < 0.005)
    {
        std::cout << "    [1/1] No standoff possible; moving directly to the target.\n";

        const bool ok = carrying
            ? transitUpright(mg, validator, target, vel_scale, acc_scale,
                             tcp_link, planning_frame)
            : moveFreeSpace(mg, validator, target, vel_scale, acc_scale);
        if (!ok) std::cout << "    [-] Direct move to the target failed.\n";
        return ok;
    }

    standoff_z = used_standoff;
    std::cout << "    [timing] standoff selection: " << sw_stage.lap()
              << " ms  (total before the arm moves: " << sw_total.ms() << " ms)\n";

    // ---- Step 3: transit ----------------------------------------------------
    std::cout << "    [1/2] Transit to standoff (z+" << standoff_z << ")...\n";

    // Upright transit regardless of payload. The 5-DOF manifold planner
    // satisfies the constraint exactly
    
    const bool upright_transit = g_upright_when_empty.load() || carrying;
    if (!carrying)
        std::cout << "    [mode] empty gripper, standoff transit = "
                  << (upright_transit ? "upright, hinted at the standoff configuration"
                                      : "joint target from the grasp state (legacy)")
                  << "\n";

    if (upright_transit)
    {
        if (!transitUpright(mg, validator, standoff, vel_scale, acc_scale,
                            tcp_link, planning_frame, TILT_TOLERANCE, 0.5, 0.7,
                            false, &standoff_state))
        {
            std::cout << "    [-] Constrained transit to standoff failed.\n";
            return false;
        }
    }
    else
    {
        mg.setStartStateToCurrentState();
        mg.clearPoseTargets();
        mg.clearPathConstraints();
        mg.setJointValueTarget(standoff_state);
        mg.setMaxVelocityScalingFactor(vel_scale);
        mg.setMaxAccelerationScalingFactor(acc_scale);

        MGI::Plan plan;
        if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            std::cout << "    [-] Transit planning failed even with a validated goal. "
                         "The start state may itself be in collision -- check RViz.\n";
            return false;
        }
        if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            std::cout << "    [-] Transit execution failed.\n";
            return false;
        }
    }
    waitForStateSettle(mg, 200);

    std::cout << "    [timing] transit (plan + execute): " << sw_stage.lap() << " ms\n";

    // Step 3b: did the arm actually arrive where it was asked to
    if (auto arrived = mg.getCurrentState(2.0))
    {
        std::string tight_joint = "?";
        const double head = minLimitHeadroom(*arrived, arm_jmg, &tight_joint);
        const double gap  = maxJointDelta(*arrived, standoff_state, arm_jmg);

        std::cout << "    [arrival] " << gap << " rad from the intended standoff"
                     " configuration; closest joint stop is '" << tight_joint
                  << "' at " << head << " rad.\n";

        if (gap > 0.05)
        {
            const double f = descentDryRun(mg, validator, *arrived, target,
                                           free_axis_tolerance);
            if (f < 0.995)
            {
                std::cout << "    [!] The arm arrived in a different configuration and the\n"
                             "        descent from it only reaches " << (f * 100.0)
                          << "%. Reconfiguring in place\n        to the verified standoff "
                             "configuration.\n";

                mg.setStartStateToCurrentState();
                mg.clearPoseTargets();
                mg.clearPathConstraints();
                if (carrying)
                    mg.setPathConstraints(uprightConstraint(tcp_link, planning_frame,
                                                            standoff.orientation));
                mg.setJointValueTarget(standoff_state);
                mg.setMaxVelocityScalingFactor(vel_scale);
                mg.setMaxAccelerationScalingFactor(acc_scale);

                MGI::Plan fix_plan;
                bool fixed = mg.plan(fix_plan) == moveit::core::MoveItErrorCode::SUCCESS &&
                             mg.execute(fix_plan) == moveit::core::MoveItErrorCode::SUCCESS;
                mg.clearPathConstraints();

                if (!fixed)
                {
                    geometry_msgs::msg::Quaternion ref = standoff.orientation;
                    fixed = moveJointInterpolated(mg, validator, standoff_state,
                                                  vel_scale, acc_scale, 30, 0.006,
                                                  carrying ? &ref : nullptr,
                                                  TILT_TOLERANCE);
                }

                if (!fixed)
                {
                    std::cout << "    [-] Could not reconfigure at the standoff.\n";
                    return false;
                }
                waitForStateSettle(mg, 200);
            }
        }
    }

    std::cout << "    [timing] arrival check: " << sw_stage.lap() << " ms\n";

    // ---- Step 4: descend ----------------------------------------------------
    std::cout << "    [2/2] Descent...\n";
    if (!moveLinear(mg, validator, target, vel_scale * 0.5, acc_scale * 0.5,
                    free_axis_tolerance))
    {
        std::cout << "    [-] Descent incomplete. Holding at standoff.\n";
        return false;
    }
    return true;
}

// Vertical ascent

static bool retreatVertical(MGI & mg,
                            IkValidator & validator,
                            double lift_z,
                            double vel_scale,
                            double acc_scale,
                            double free_axis_tolerance = 0.0)
{
    geometry_msgs::msg::Pose up = mg.getCurrentPose().pose;
    up.position.z += lift_z;

    std::cout << "    [ascent] lifting " << (lift_z * 1000.0) << " mm...\n";
    if (!moveLinear(mg, validator, up, vel_scale, acc_scale, free_axis_tolerance))
    {
        std::cout << "    [-] Ascent incomplete. The arm is still low -- do not\n"
                     "        issue a free-space move until this is resolved.\n";
        return false;
    }
    return true;
}

//  MOVEIT TASK CONSTRUCTOR PIPELINE

static bool executeLiquidTransferTask(int target_marker,
                                      tf2_ros::Buffer & tf_buffer,
                                      const std::string & planning_frame,
                                      const rclcpp::Node::SharedPtr & node,
                                      const std::string & tcp_link)
{
    const std::string tube_name = "tube_" + std::to_string(target_marker);
    std::cout << "\n>>> [MTC] Building pipeline for " << tube_name << "...\n";

    ScenePause pause;

    double tube_mx = 0.0, tube_my = 0.0, mixer_mx = 0.0, mixer_my = 0.0;
    if (!lookupMarkerXY(tf_buffer, planning_frame, target_marker, tube_mx, tube_my))
    {
        std::cout << "[-] MTC: TF lookup for marker_" << target_marker << " failed.\n";
        return false;
    }
    if (!lookupMarkerXY(tf_buffer, planning_frame, MIXER_MARKER, mixer_mx, mixer_my))
    {
        std::cout << "[-] MTC: TF lookup for marker_6 (mixer) failed.\n";
        return false;
    }

    const double grasp_x = tube_mx;
    const double grasp_y = tube_my + GRASP_Y_OFFSET;

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);
    const geometry_msgs::msg::Quaternion q_upright_msg = tf2::toMsg(q_upright);

    // ---- 2. Task and solvers -----------------------------------------------

    static std::unique_ptr<mtc::Task> task_holder;
    task_holder = std::make_unique<mtc::Task>();
    mtc::Task & task = *task_holder;

    task.stages()->setName("liquid transfer " + tube_name);
    task.loadRobotModel(node);

    task.setProperty("group", std::string("arm_group"));
    task.setProperty("ik_frame", tcp_link);

    auto planner_ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
    planner_ompl->setPlannerId("RRTConnectkConfigDefault");
    planner_ompl->setMaxVelocityScalingFactor(VEL_SCALE_TRANSIT);
    planner_ompl->setMaxAccelerationScalingFactor(ACC_SCALE_TRANSIT);
    planner_ompl->setTimeout(10.0);

    auto planner_cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    planner_cartesian->setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
    planner_cartesian->setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);
    planner_cartesian->setStepSize(0.005);
    planner_cartesian->setJumpThreshold(5.0);

    auto planner_gripper = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    const auto * gripper_jmg = task.getRobotModel()->getJointModelGroup("gripper");
    if (!gripper_jmg)
    {
        std::cout << "[-] MTC: no 'gripper' joint model group in the SRDF.\n";
        return false;
    }
    const std::vector<std::string> gripper_links = gripper_jmg->getLinkModelNames();

    // ---- 3. Stages ----------------------------------------------------------

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
        auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", planner_gripper);
        s->setGroup("gripper");
        s->setGoal("open");
        task.add(std::move(s));
    }

    // Pre-grasp: above the tube.
    geometry_msgs::msg::PoseStamped pregrasp;
    pregrasp.header.frame_id  = planning_frame;
    pregrasp.pose.position.x  = grasp_x;
    pregrasp.pose.position.y  = grasp_y;
    pregrasp.pose.position.z  = GRASP_Z + APPROACH_DIST;
    pregrasp.pose.orientation = q_upright_msg;
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", planner_ompl);
        s->setGroup("arm_group");
        s->setGoal(pregrasp);
        task.add(std::move(s));
    }

    {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-tube contact");
        s->allowCollisions(tube_name, gripper_links, true);
        task.add(std::move(s));
    }

    // Descend a deterministic distance
    {
        auto s = std::make_unique<mtc::stages::MoveRelative>("approach tube", planner_cartesian);
        s->setGroup("arm_group");
        s->setIKFrame(tcp_link);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = planning_frame;
        dir.vector.z = -1.0;
        s->setDirection(dir);
        s->setMinMaxDistance(APPROACH_DIST - 0.01, APPROACH_DIST);
        task.add(std::move(s));
    }

    // Attach before closing
    {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach tube");
        s->attachObject(tube_name, tcp_link);
        task.add(std::move(s));
    }
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("close gripper", planner_gripper);
        s->setGroup("gripper");
        s->setGoal("closed");
        task.add(std::move(s));
    }

    // Straight vertical lift
    {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lift tube", planner_cartesian);
        s->setGroup("arm_group");
        s->setIKFrame(tcp_link);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = planning_frame;
        dir.vector.z = 1.0;
        s->setDirection(dir);
        s->setMinMaxDistance(LIFT_DIST - 0.02, LIFT_DIST);
        task.add(std::move(s));
    }

    // Transit to the mixer held upright.
    geometry_msgs::msg::PoseStamped pour_pose;
    pour_pose.header.frame_id  = planning_frame;
    mixerPourPose(mixer_mx, mixer_my,
                  pour_pose.pose.position.x,
                  pour_pose.pose.position.y,
                  pour_pose.pose.position.z);
    pour_pose.pose.orientation = q_upright_msg;
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("move to mixer", planner_ompl);
        s->setGroup("arm_group");
        s->setGoal(pour_pose);
        s->setPathConstraints(
            uprightConstraint(tcp_link, planning_frame, q_upright_msg));

        task.add(std::move(s));
    }

    // ---- 4. Plan and execute ------------------------------------------------
    try
    {
        if (!task.plan(3))
        {
            std::cout << "[-] MTC found no valid pipeline.\n"
                         "    Open the 'Motion Planning Tasks' panel in RViz to see\n"
                         "    which stage failed and why.\n";
            return false;
        }
    }
    catch (const mtc::InitStageException & e)
    {
        std::cout << "[-] MTC init error: " << e << "\n";
        return false;
    }
    catch (const std::exception & e)
    {
        std::cout << "[-] MTC planning threw: " << e.what() << "\n";
        return false;
    }

    std::cout << ">>> [MTC] Plan found (" << task.solutions().size()
              << " solutions). Executing best...\n";

    const auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        std::cout << "[-] MTC execution failed, error code " << result.val << "\n";
        return false;
    }

    return true;
}

//  MAIN

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("liquid_handler_master");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    MGI arm_interface(node, "arm_group");
    MGI gripper_interface(node, "gripper");

    IkValidator ik_validator(node);

    const std::string planning_frame = arm_interface.getPlanningFrame();
    const std::string tcp_link       = arm_interface.getEndEffectorLink();

    int allowed_tube_id = -1;  
    std::vector<std::string> gripper_links;
    if (const auto * gjmg = arm_interface.getRobotModel()->getJointModelGroup("gripper"))
    {
        gripper_links = gjmg->getLinkModelNames();
        if (std::find(gripper_links.begin(), gripper_links.end(), tcp_link) == gripper_links.end())
            gripper_links.push_back(tcp_link);
    }
    else
        std::cout << "[!] No 'gripper' joint model group; collision allowances disabled.\n";


    std::cout << ">>> [INIT] Planning frame: " << planning_frame
              << " | TCP link: " << tcp_link << "\n";

    auto tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    std::cout << ">>> [INIT] Waiting 2 s for the TF tree to populate...\n";
    rclcpp::sleep_for(std::chrono::seconds(2));

    setupCollisionObjects(planning_frame, *tf_buffer);

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);

    // Reduced-space transit planner
    g_use_manifold = node->declare_parameter<bool>("use_manifold", true);
    if (g_use_manifold)
    {
        try
        {
            g_manifold = std::make_unique<my_robot_control::ManifoldPlanner>();
            my_robot_control::ManifoldPlanner::Options mopt;
            mopt.model = arm_interface.getRobotModel();
            mopt.tcp_link = tcp_link;
            mopt.planning_time = node->declare_parameter<double>("manifold_time", 2.0);
            mopt.lock_arm_angle = node->declare_parameter<bool>("manifold_lock_arm_angle", false);
            g_manifold->initialize(mopt);
            std::cout << ">>> [INIT] 5-DOF manifold transit planner ready"
                      << (mopt.lock_arm_angle ? " (arm angle locked)" : "")
                      << ". Fallback: constrained 7-DOF.\n";
        }
        catch (const std::exception & e)
        {
            std::cout << "[!] [INIT] Manifold planner unavailable (" << e.what()
                      << "); using constrained 7-DOF planning only.\n";
            g_manifold.reset();
            g_use_manifold = false;
        }
    }
    else
        std::cout << ">>> [INIT] Manifold planner disabled by parameter.\n";

    // ---- Planner defaults ---------------------------------------------------
    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(10.0);
    arm_interface.setNumPlanningAttempts(5);
    arm_interface.setWorkspace(-1.0, -1.0, -0.1, 1.0, 1.0, 1.0);
    arm_interface.setGoalPositionTolerance(0.005);
    arm_interface.setGoalOrientationTolerance(0.02);

    geometry_msgs::msg::Pose home_pose;
    home_pose.position.x    = -0.15;
    home_pose.position.y    = -0.15;
    home_pose.position.z    = 0.20;
    home_pose.orientation.x = q_upright.x();
    home_pose.orientation.y = q_upright.y();
    home_pose.orientation.z = q_upright.z();
    home_pose.orientation.w = q_upright.w();

    std::cout << ">>> [INIT] Moving to home pose...\n";
    if (!moveFreeSpace(arm_interface, ik_validator, home_pose,
                       VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT))
    {
        std::cout << "[-] [INIT] Failed to reach the home pose. Aborting.\n";
        rclcpp::shutdown();
        spinner.join();
        return -1;
    }
    waitForStateSettle(arm_interface);

    // ---- Command intake -----------------------------------------------------
    auto gui_sub = node->create_subscription<std_msgs::msg::String>(
        "/gui_commands", 10,
        [](const std_msgs::msg::String::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            command_queue.push(msg->data);
            std::cout << "[QUEUE] " << msg->data
                      << " (pending: " << command_queue.size() << ")\n";
        });

    auto status_pub = node->create_publisher<std_msgs::msg::String>("/gui_status", 10);

    std::string attached_tube;
    int current_marker_id = -1;

    // ---- Background scene updater -------------------------------------------
    moveit::planning_interface::PlanningSceneInterface dynamic_scene_interface;
    auto dynamic_updater = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [&]()
        {
            if (scene_updates_paused.load()) return;

            std::vector<moveit_msgs::msg::CollisionObject> updates;
            const int held = attached_marker_id.load();

            for (int i = 1; i <= NUM_TUBES; ++i)
            {
                if (i == held) continue;   // never fight the attached copy
                double mx = 0.0, my = 0.0;
                if (lookupMarkerXY(*tf_buffer, planning_frame, i, mx, my))
                    updates.push_back(makeTube(i, planning_frame, mx, my));
            }

            double mixer_x = 0.0, mixer_y = 0.0;
            if (lookupMarkerXY(*tf_buffer, planning_frame, MIXER_MARKER, mixer_x, mixer_y))
                updates.push_back(makeMixer(planning_frame, mixer_x, mixer_y));

            if (!updates.empty())
                dynamic_scene_interface.applyCollisionObjects(updates);
        });

    std::cout << "\n>>> Ready. Listening on /gui_commands.\n";

    //  MAIN EXECUTION LOOP
    
    while (rclcpp::ok())
    {
        std::string line;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (!command_queue.empty())
            {
                line = command_queue.front();
                command_queue.pop();
            }
        }

        if (line.empty())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::cout << "\n[EXECUTING] " << line << "\n";

        CommandReport report{ status_pub, line, false };

        // ---- Quit -----------------------------------------------------------
        if (line == "q" || line == "Q") { report.ok = true; break; }

        // ---- MTC pipeline ---------------------------------------------------
        if (line.rfind("TASK", 0) == 0 || line.rfind("task", 0) == 0)
        {
            int marker_id = -1;
            try
            {
                marker_id = std::stoi(line.substr(4));   // handles "TASK3" and "TASK 3"
            }
            catch (const std::exception &)
            {
                std::cout << "    [-] Bad command. Use: TASK <1-" << NUM_TUBES << ">\n";
                continue;
            }

            if (marker_id < 1 || marker_id > NUM_TUBES)
            {
                std::cout << "    [-] TASK marker must be 1-" << NUM_TUBES << ".\n";
                continue;
            }
            if (!attached_tube.empty())
            {
                std::cout << "    [-] Already holding " << attached_tube
                          << ". Send 'o' to release first.\n";
                continue;
            }

            const bool ok = executeLiquidTransferTask(
                marker_id, *tf_buffer, planning_frame, node, tcp_link);
            report.ok = ok;

            if (ok)
            {
                attached_tube     = "tube_" + std::to_string(marker_id);
                current_marker_id = marker_id;
                attached_marker_id.store(marker_id);
                std::cout << ">>> Task complete. Holding " << attached_tube << ".\n";
            }
            else
            {
                std::cout << "    [!] Task failed or aborted.\n";
            }
            waitForStateSettle(arm_interface);
            continue;
        }

        if (line == "u" || line == "U")
        {
            const bool now = !g_upright_when_empty.load();
            g_upright_when_empty.store(now);
            std::cout << ">>> Empty-gripper standoff transit is now "
                      << (now ? "UPRIGHT, hinted at the verified standoff "
                                "configuration (default)"
                              : "a joint target from the grasp state (legacy)")
                      << ".\n";
            report.ok = true;
            continue;
        }

        if (line == "h" || line == "H")
        {
            const double v = attached_tube.empty() ? VEL_SCALE_TRANSIT : VEL_SCALE_LIQUID;
            const double a = attached_tube.empty() ? ACC_SCALE_TRANSIT : ACC_SCALE_LIQUID;
            const bool ok = attached_tube.empty()
                ? moveFreeSpace(arm_interface, ik_validator, home_pose, v, a)
                : transitUpright(arm_interface, ik_validator, home_pose, v, a,
                                 tcp_link, planning_frame);
            report.ok = ok;
            if (ok)
                std::cout << ">>> Home.\n";
            else
                std::cout << "    [-] Could not reach home.\n";
            arm_interface.clearPathConstraints();
            waitForStateSettle(arm_interface);
            continue;
        }

        // Open gripper
        if (line == "o" || line == "O")
        {
            std::cout << ">>> Opening gripper...\n";
            gripper_interface.setNamedTarget("open");
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                if (!attached_tube.empty())
                {
                    // detachObject() returns the object to the world
                    const int released_id = attached_marker_id.load();
                    arm_interface.detachObject(attached_tube);
                    std::cout << ">>> Detached " << attached_tube << ".\n";
                    attached_tube.clear();
                    rclcpp::sleep_for(std::chrono::milliseconds(200));

                    // Vertical ascent
                    waitForStateSettle(arm_interface, 200);
                    if (!retreatVertical(arm_interface, ik_validator, LIFT_DIST,
                                         VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT,
                                         TOOL_AXIS_FREEDOM))
                        std::cout << "    [!] Could not lift clear of the tube.\n";

                    if (!gripper_links.empty() && released_id >= 1)
                        ik_validator.allowCollisions("tube_" + std::to_string(released_id),
                                                     gripper_links, false);
                    allowed_tube_id = -1;
                    attached_marker_id.store(-1);
                    waitForStateSettle(arm_interface, 200);
                }
                report.ok = true;
            }
            else
            {
                std::cout << "    [-] Gripper failed to open.\n";
            }
            continue;
        }

        // Close gripper (manual grasp)
        if (line == "c" || line == "C")
        {
            std::cout << ">>> Closing gripper...\n";

            if (current_marker_id >= 1 && current_marker_id <= NUM_TUBES && attached_tube.empty())
            {
                const std::string tube_id = "tube_" + std::to_string(current_marker_id);

                const auto eef = arm_interface.getCurrentPose().pose;
                moveit_msgs::msg::CollisionObject restored;
                restored.id = tube_id;
                restored.header.frame_id = planning_frame;
                restored.primitives.resize(1);
                restored.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                restored.primitives[0].dimensions = { TUBE_HEIGHT, TUBE_RADIUS };
                restored.primitive_poses.resize(1);
                restored.primitive_poses[0].position.x = eef.position.x;
                restored.primitive_poses[0].position.y = eef.position.y + TUBE_TCP_Y_OFFSET;
                restored.primitive_poses[0].position.z = eef.position.z + TUBE_TCP_Z_OFFSET;
                restored.primitive_poses[0].orientation.w = 1.0;
                restored.operation = restored.ADD;

                attached_marker_id.store(current_marker_id);   // stop the updater first
                dynamic_scene_interface.applyCollisionObject(restored);
                rclcpp::sleep_for(std::chrono::milliseconds(150));

                std::vector<std::string> touch_links;
                if (const auto * jmg = gripper_interface.getRobotModel()->getJointModelGroup("gripper"))
                    touch_links = jmg->getLinkModelNames();

                arm_interface.attachObject(tube_id, tcp_link, touch_links);
                attached_tube = tube_id;
                std::cout << ">>> Attached " << tube_id << " to " << tcp_link << ".\n";
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }

            gripper_interface.setNamedTarget("closed");
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                std::cout << ">>> Gripper closed.\n";
                if (!attached_tube.empty())
                {
                    waitForStateSettle(arm_interface, 200);
                    if (!retreatVertical(arm_interface, ik_validator, LIFT_DIST,
                                         VEL_SCALE_LIQUID, ACC_SCALE_LIQUID,
                                         TOOL_AXIS_FREEDOM))
                        std::cout << "    [!] Tube is grasped but still down in the rack.\n";
                    else
                        std::cout << ">>> Lifted clear of the rack.\n";
                    waitForStateSettle(arm_interface, 200);
                }
                report.ok = true;
            }
            else
            {
                std::cout << "    [-] Gripper failed to close. Reverting attach.\n";
                if (!attached_tube.empty())
                {
                    arm_interface.detachObject(attached_tube);
                    attached_tube.clear();
                    attached_marker_id.store(-1);
                }
            }
            continue;
        }

        // Pour
        if (line == "p" || line == "P")
        {
            if (attached_tube.empty())
            {
                std::cout << "    [-] Nothing attached; refusing to pour.\n";
                continue;
            }

            std::cout << ">>> Pouring...\n";
            auto state = arm_interface.getCurrentState(2.0);
            if (!state)
            {
                std::cout << "    [-] Could not read robot state.\n";
                continue;
            }

            const auto * jmg = state->getJointModelGroup("arm_group");
            std::vector<double> joints;
            state->copyJointGroupPositions(jmg, joints);
            if (joints.empty()) continue;

            const double upright = joints.back();

            arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
            arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

            joints.back() = upright + M_PI / 2.0;
            arm_interface.setJointValueTarget(joints);
            if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                waitForStateSettle(arm_interface);
                std::cout << "    [+] Tilted. Draining...\n";
                rclcpp::sleep_for(std::chrono::seconds(1));

                joints.back() = upright;
                arm_interface.setJointValueTarget(joints);
                if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    waitForStateSettle(arm_interface);
                    std::cout << ">>> Poured.\n";
                    report.ok = true;
                }
                else
                {
                    std::cout << "    [!] Failed to return upright -- tube may still be tilted.\n";
                }
            }
            else
            {
                std::cout << "    [-] Could not execute the pour.\n";
            }
            continue;
        }

        // Manual move: "m<N>" or raw "x y z"
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double standoff = STANDOFF_TUBE;

        if (line[0] == 'm' || line[0] == 'M')
        {
            int marker_id = -1;
            std::stringstream ss(line.substr(1));
            if (!(ss >> marker_id))
            {
                std::cout << "    [-] Bad marker command.\n";
                continue;
            }
            current_marker_id = marker_id;

            if (marker_id == 0)
            {
                tx = 0.0; ty = -0.065; tz = 0.20;
            }
            else
            {
                double mx = 0.0, my = 0.0;
                if (lookupMarkerXY(*tf_buffer, planning_frame, marker_id, mx, my))
                {
                    tx = mx;
                    if (marker_id == MIXER_MARKER)
                    {
                        mixerPourPose(mx, my, tx, ty, tz);
                        standoff = STANDOFF_MIXER;
                    }
                    else
                    {
                        ty = my + GRASP_Y_OFFSET;
                        tz = GRASP_Z;
                    }
                }
                else
                {
                    std::cout << "    [-] No TF for marker_" << marker_id
                              << "; using hardcoded fallback.\n";
                    if (marker_id >= 1 && marker_id <= NUM_TUBES)
                    {
                        tx = TUBE_FALLBACK_X[marker_id - 1];
                        ty = TUBE_FALLBACK_Y;
                        tz = TUBE_FALLBACK_Z;
                    }
                    else if (marker_id == MIXER_MARKER)
                    {
                        mixerPourPose(MIXER_FALLBACK_MARKER_X, MIXER_FALLBACK_MARKER_Y,
                                      tx, ty, tz);
                        standoff = STANDOFF_MIXER;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else
        {
            std::stringstream ss(line);
            if (!(ss >> tx >> ty >> tz))
            {
                std::cout << "    [-] Unrecognised command.\n";
                continue;
            }
            current_marker_id = -1;
        }

        // Execute the manual hybrid move
        std::cout << "--- Moving to (" << tx << ", " << ty << ", " << tz << ") ---\n";

        geometry_msgs::msg::Pose target;
        target.position.x    = tx;
        target.position.y    = ty;
        target.position.z    = tz;
        target.orientation.x = q_upright.x();
        target.orientation.y = q_upright.y();
        target.orientation.z = q_upright.z();
        target.orientation.w = q_upright.w();

        const double v = attached_tube.empty() ? VEL_SCALE_TRANSIT : VEL_SCALE_LIQUID;
        const double a = attached_tube.empty() ? ACC_SCALE_TRANSIT : ACC_SCALE_LIQUID;

        const bool carrying = !attached_tube.empty();


        if (!carrying && current_marker_id >= 1 && current_marker_id <= NUM_TUBES &&
            !gripper_links.empty() && allowed_tube_id != current_marker_id)
        {
            if (allowed_tube_id >= 1)   // revoke the previous one first
                ik_validator.allowCollisions("tube_" + std::to_string(allowed_tube_id),
                                             gripper_links, false);
            attached_marker_id.store(current_marker_id);   // freeze the updater too
            ik_validator.allowCollisions("tube_" + std::to_string(current_marker_id),
                                         gripper_links, true);
            allowed_tube_id = current_marker_id;
        }

        const bool reached = withRetries("approach", 3, [&]() {
            return approachTarget(arm_interface, ik_validator, target, standoff,
                                  v, a, TOOL_AXIS_FREEDOM,
                                  carrying, tcp_link, planning_frame);
        });

        if (!reached)
        {
            std::cout << "    [!] RECOVERY: returning home...\n";
            const bool home_ok = carrying
                ? transitUpright(arm_interface, ik_validator, home_pose,
                                 VEL_SCALE_LIQUID, ACC_SCALE_LIQUID,
                                 tcp_link, planning_frame, 0.25, 0.45, 0.60)
                : moveFreeSpace(arm_interface, ik_validator, home_pose,
                                VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT);
            if (!home_ok)
                std::cout << "    [!] CRITICAL: could not reach home. Arm may be trapped.\n";
        }
        else
        {
            std::cout << ">>> Target reached.\n";
            report.ok = true;
        }

        arm_interface.clearPathConstraints();
        waitForStateSettle(arm_interface);
    }

    std::cout << "\n>>> Shutting down.\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
