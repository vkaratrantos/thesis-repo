#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip> // Για ωραία εκτύπωση αριθμών

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Συνάρτηση για να τυπώνει τη θέση καθαρά
void printPose(const geometry_msgs::msg::Pose& pose) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[ΤΩΡΑ]: X=" << pose.position.x 
              << ", Y=" << pose.position.y 
              << ", Z=" << pose.position.z << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("vial_pro_system");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group");
  MoveGroupInterface gripper_interface(node, "gripper");

  // Ρυθμίσεις Ταχύτητας (Γρήγορο για απόκριση)
  arm_interface.setMaxVelocityScalingFactor(1.0);
  arm_interface.setMaxAccelerationScalingFactor(1.0);
  arm_interface.setPlanningTime(5.0); // Δίνουμε 5 δευτερόλεπτα να σκεφτεί

  // --- ΠΡΟΣΘΗΚΗ ΠΑΤΩΜΑΤΟΣ (Προαιρετικό - αν σε ενοχλεί, σχολίασέ το) ---
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world"; // ή "base_link"
  collision_object.id = "floor";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {2.0, 2.0, 0.02};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.z = -0.05; // 5 πόντους κάτω για ασφάλεια
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObjects({collision_object});
  // -------------------------------------------------------------------

  RCLCPP_INFO(node->get_logger(), "System Ready. Like RViz.");

  while (rclcpp::ok()) {
    std::cout << "\n------------------------------------------------" << std::endl;
    
    // 1. ΔΙΑΒΑΣΜΑ ΤΡΕΧΟΥΣΑΣ ΘΕΣΗΣ (Αυτό έλειπε!)
    // Έτσι ξέρεις ακριβώς πού είναι το ρομπότ πριν δώσεις εντολή.
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    printPose(current_pose);

    std::cout << "1. Go to XYZ (RViz Style - Smart Plan)" << std::endl;
    std::cout << "2. Gripper Open/Close" << std::endl;
    std::cout << "Επιλογή: " << std::flush;

    std::string choice;
    std::getline(std::cin, choice);

    if (choice == "1") {
        std::cout << "Δώσε Στόχο X Y Z (π.χ. 0.2 0.0 0.25): ";
        std::string line;
        std::getline(std::cin, line);
        std::stringstream ss(line);
        double tx, ty, tz;
        
        if (ss >> tx >> ty >> tz) {
            // ΟΡΙΣΜΟΣ ΣΤΟΧΟΥ
            geometry_msgs::msg::Pose target_pose = current_pose; // Ξεκινάμε με αντιγραφή του τωρινού
            target_pose.position.x = tx;
            target_pose.position.y = ty;
            target_pose.position.z = tz;
            
            // Κρατάμε τον ΙΔΙΟ προσανατολισμό με τώρα (όπως όταν σέρνεις την μπίλια)
            // Αν θες να το ισιώσεις οριζόντια, βγάλε τα σχόλια από κάτω:
            /*
            tf2::Quaternion q;
            q.setRPY(0, 1.57, 0); 
            target_pose.orientation = tf2::toMsg(q);
            */

            arm_interface.setPoseTarget(target_pose);

            // PLANNING (Όχι Cartesian - Κανονικό Planning όπως το RViz)
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (arm_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                std::cout << ">>> Σχέδιο βρέθηκε! Εκτέλεση..." << std::endl;
                arm_interface.execute(my_plan);
            } else {
                std::cout << ">>> ΑΠΟΤΥΧΙΑ: Το σημείο είναι εκτός εμβέλειας ή χτυπάει κάπου." << std::endl;
            }
        }
    } 
    else if (choice == "2") {
        std::cout << "1. Open | 2. Close: ";
        std::string g_choice;
        std::getline(std::cin, g_choice);
        
        // Χρήση Joint Target για το Gripper (πιο αξιόπιστο)
        std::vector<double> gripper_joints;
        if (g_choice == "1") gripper_joints = {-0.7}; // Open (περίπου)
        else gripper_joints = {0.0};   // Close

        gripper_interface.setJointValueTarget(gripper_joints);
        gripper_interface.move();
    }
  }

  rclcpp::shutdown();
  return 0;
}
