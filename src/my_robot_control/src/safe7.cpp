#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>

void printPose(const geometry_msgs::msg::Pose& pose) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[ΤΡΕΧΟΥΣΑ ΘΕΣΗ]: X=" << pose.position.x 
              << ", Y=" << pose.position.y 
              << ", Z=" << pose.position.z << std::endl;
}

double getDistance(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2) {
    return std::sqrt(std::pow(p1.position.x - p2.position.x, 2) +
                     std::pow(p1.position.y - p2.position.y, 2) +
                     std::pow(p1.position.z - p2.position.z, 2));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_move_node");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group"); 
  MoveGroupInterface gripper_interface(node, "gripper"); 

  arm_interface.setMaxVelocityScalingFactor(0.3); 
  arm_interface.setMaxAccelerationScalingFactor(0.3);

  // --- ΠΡΟΣΘΗΚΗ ΠΑΤΩΜΑΤΟΣ ---
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = arm_interface.getPlanningFrame();
  collision_object.id = "floor";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {2.0, 2.0, 0.02};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.z = -0.05; 
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObjects({collision_object});

  tf2::Quaternion q_horizontal;
  q_horizontal.setRPY(0.0, 1.5708, 0.0); 
  geometry_msgs::msg::Quaternion q_horizontal_msg;
  q_horizontal_msg.x = q_horizontal.x(); q_horizontal_msg.y = q_horizontal.y();
  q_horizontal_msg.z = q_horizontal.z(); q_horizontal_msg.w = q_horizontal.w();

  auto try_incremental_search = [&](double target_x, double target_y, double target_z) -> bool {
      geometry_msgs::msg::Pose target;
      target.position.x = target_x; target.position.y = target_y; target.position.z = target_z;
      target.orientation = q_horizontal_msg;

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> noise_dist(-0.06, 0.06); 
      std::uniform_real_distribution<> step_dist(0.03, 0.12);   

      // Εργαλεία για τον Απεγκλωβισμό
      std::uniform_real_distribution<> escape_xy_dist(-0.15, 0.15); 
      std::uniform_real_distribution<> escape_z_dist(0.0, 0.15); 

      std::cout << "\n    [ΕΝΑΡΞΗ] Αυξητική Αναζήτηση με Γρήγορο Auto-Escape..." << std::endl;

      int max_total_steps = 300; 
      int step_count = 0;

      while (step_count < max_total_steps) {
          geometry_msgs::msg::Pose current = arm_interface.getCurrentPose().pose;
          double current_dist = getDistance(current, target);

          if (current_dist < 0.01) {
              std::cout << "    [ΕΠΙΤΥΧΙΑ] Το ρομπότ έφτασε ακριβώς στο σημείο!" << std::endl;
              return true;
          }

          // 1. Δοκιμή απόλυτης ευθείας για το υπόλοιπο
          std::vector<geometry_msgs::msg::Pose> direct_wp = {target};
          moveit_msgs::msg::RobotTrajectory traj;
          if (arm_interface.computeCartesianPath(direct_wp, 0.01, 1.5, traj) >= 0.95) {
              std::cout << "    -> Βρέθηκε καθαρή ευθεία για το υπόλοιπο! Εκτέλεση..." << std::endl;
              moveit::planning_interface::MoveGroupInterface::Plan plan;
              plan.trajectory_ = traj;
              arm_interface.execute(plan);
              return true;
          }

          // 2. ΓΡΗΓΟΡΗ αναζήτηση ενδιάμεσου σημείου
          bool found_intermediate = false;
          int max_attempts = 10; // ΑΛΛΑΓΗ: Μόλις 10 προσπάθειες πριν τα παρατήσει και κάνει Escape!

          for (int i = 0; i < max_attempts; ++i) {
              geometry_msgs::msg::Pose candidate = current;
              
              double dx = target.position.x - current.position.x;
              double dy = target.position.y - current.position.y;
              double dz = target.position.z - current.position.z;
              
              double step = step_dist(gen); 

              candidate.position.x += (dx / current_dist) * step + noise_dist(gen);
              candidate.position.y += (dy / current_dist) * step + noise_dist(gen);
              candidate.position.z += (dz / current_dist) * step + noise_dist(gen);
              candidate.orientation = q_horizontal_msg;

              if (candidate.position.z < -0.02) continue;

              // Αν μας φέρνει πιο κοντά...
              if (getDistance(candidate, target) < current_dist) {
                  std::vector<geometry_msgs::msg::Pose> cand_wp = {candidate};
                  moveit_msgs::msg::RobotTrajectory cand_traj;
                  if (arm_interface.computeCartesianPath(cand_wp, 0.01, 1.5, cand_traj) >= 0.95) {
                      std::cout << "       [+] Νέο ενδιάμεσο σημείο (Απόσταση: " << current_dist << "m). Προώθηση..." << std::endl;
                      moveit::planning_interface::MoveGroupInterface::Plan plan;
                      plan.trajectory_ = cand_traj;
                      arm_interface.execute(plan); 
                      found_intermediate = true;
                      break; 
                  }
              }
          }

          // 3. ΜΗΧΑΝΙΣΜΟΣ ΑΠΕΓΚΛΩΒΙΣΜΟΥ 
          if (!found_intermediate) {
              std::cout << "    [!] Ενεργοποίηση Γρήγορης Διαδικασίας Απεγκλωβισμού..." << std::endl;
              
              bool escaped = false;
              int escape_attempts = 30; // ΑΛΛΑΓΗ: Μειωμένο και αυτό για αστραπιαία αντίδραση
              
              for (int e = 0; e < escape_attempts; ++e) { 
                  geometry_msgs::msg::Pose escape_candidate = current;
                  
                  escape_candidate.position.x += escape_xy_dist(gen);
                  escape_candidate.position.y += escape_xy_dist(gen);
                  escape_candidate.position.z += escape_z_dist(gen); 
                  escape_candidate.orientation = q_horizontal_msg; 

                  if (escape_candidate.position.z < 0.0) continue;

                  std::vector<geometry_msgs::msg::Pose> esc_wp = {escape_candidate};
                  moveit_msgs::msg::RobotTrajectory esc_traj;
                  if (arm_interface.computeCartesianPath(esc_wp, 0.01, 1.5, esc_traj) >= 0.95) {
                      std::cout << "       [!] Βρέθηκε οδός διαφυγής! Αναδίπλωση βραχίονα..." << std::endl;
                      moveit::planning_interface::MoveGroupInterface::Plan plan;
                      plan.trajectory_ = esc_traj;
                      arm_interface.execute(plan);
                      escaped = true;
                      break; 
                  }
              }

              if (!escaped) {
                  std::cout << "    [ΣΦΑΛΜΑ] Το ρομπότ εγκλωβίστηκε ολοκληρωτικά." << std::endl;
                  return false;
              }
          }

          step_count++;
      }

      std::cout << "    [ΣΦΑΛΜΑ] Εξαντλήθηκαν τα συνολικά βήματα." << std::endl;
      return false;
  };

  RCLCPP_INFO(node->get_logger(), "Εκκίνηση διαδικασίας...");

  std::cout << ">>> Μετάβαση σε αρχική θέση Home (0.0, 0.2, 0.25)..." << std::endl;
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.0; start_pose.position.y = 0.2; start_pose.position.z = 0.25; 
  start_pose.orientation = q_horizontal_msg;
  arm_interface.setPoseTarget(start_pose);
  arm_interface.move();

  std::cout << ">>> Σύστημα Έτοιμο: Ενεργός Αλγόριθμος Incremental Greedy Search (React: Fast)." << std::endl;

  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    arm_interface.setStartStateToCurrentState();
    printPose(arm_interface.getCurrentPose().pose);

    std::cout << "Εντολές: P x y z, O, C, ή απλά x y z\nΕντολή: ";
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) continue;

    if (line == "O" || line == "o") { gripper_interface.setNamedTarget("open"); gripper_interface.move(); continue; } 
    if (line == "C" || line == "c") { gripper_interface.setNamedTarget("closed"); gripper_interface.move(); continue; }

    if (line[0] == 'P' || line[0] == 'p') {
        std::stringstream ss(line.substr(1)); double px, py, pz;
        if (ss >> px >> py >> pz) {
            std::cout << "\n>>> ΕΚΤΕΛΕΣΗ PICK ΣΤΟ (" << px << ", " << py << ", " << pz << ")" << std::endl;
            if(!try_incremental_search(px, py, pz + 0.10)) continue;
            gripper_interface.setNamedTarget("open"); gripper_interface.move();
            if(!try_incremental_search(px, py, pz)) continue;
            gripper_interface.setNamedTarget("closed"); gripper_interface.move();
            try_incremental_search(px, py, pz + 0.10);
        }
    } else {
        std::stringstream ss(line); double tx, ty, tz;
        if (ss >> tx >> ty >> tz) try_incremental_search(tx, ty, tz);
    }
  }
  rclcpp::shutdown();
  return 0;
}
