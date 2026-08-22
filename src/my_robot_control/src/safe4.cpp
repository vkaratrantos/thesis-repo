#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

void printPose(const geometry_msgs::msg::Pose& pose) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[ΤΡΕΧΟΥΣΑ ΘΕΣΗ]: X=" << pose.position.x 
              << ", Y=" << pose.position.y 
              << ", Z=" << pose.position.z << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("relaxed_move_node");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group"); 
  MoveGroupInterface gripper_interface(node, "gripper"); 

  arm_interface.setMaxVelocityScalingFactor(0.5); 
  arm_interface.setMaxAccelerationScalingFactor(0.5);
  
  // --- ΧΑΛΑΡΩΣΗ ΑΠΑΙΤΗΣΕΩΝ ---
  arm_interface.setPlanningTime(15.0); // Του δίνουμε άπλετο χρόνο
  arm_interface.setNumPlanningAttempts(10); // Του λέμε να δοκιμάσει 10 διαφορετικές προσεγγίσεις πριν τα παρατήσει
  
  // Ορίζουμε τον προεπιλεγμένο Planner (καλός για ελεύθερη κίνηση χωρίς περιορισμούς)
  arm_interface.setPlannerId("RRTConnectkConfigDefault");

  // --- ΠΡΟΣΘΗΚΗ ΠΑΤΩΜΑΤΟΣ (Το κρατάμε για ασφάλεια) ---
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
  // -------------------------------------------------------------------

  RCLCPP_INFO(node->get_logger(), "Σύστημα Έτοιμο. Αναμονή για συντεταγμένες...");

  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    
    // Διαβάζουμε και τυπώνουμε την τρέχουσα θέση
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    printPose(current_pose);

    std::cout << "Δώσε Στόχο X Y Z χωρισμένα με κενό (π.χ. 0.3 0.1 0.2): ";
    std::string line;
    std::getline(std::cin, line);
    std::stringstream ss(line);
    double tx, ty, tz;
    
    if (ss >> tx >> ty >> tz) {
        
        // ΟΡΙΖΟΥΜΕ ΜΟΝΟ ΤΗ ΘΕΣΗ (Position) ΚΑΙ ΑΓΝΟΟΥΜΕ ΤΟΝ ΠΡΟΣΑΝΑΤΟΛΙΣΜΟ
        // Αυτό δίνει την απόλυτη ελευθερία στο ρομπότ να περιστρέψει τις αρθρώσεις του όπως θέλει
        arm_interface.setPositionTarget(tx, ty, tz);

        // PLANNING & EXECUTION
        std::cout << ">>> Υπολογισμός διαδρομής (ΧΩΡΙΣ ΠΕΡΙΟΡΙΣΜΟΥΣ)..." << std::endl;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            std::cout << ">>> Σχέδιο βρέθηκε! Το ρομπότ κινείται..." << std::endl;
            arm_interface.execute(my_plan);
        } else {
            std::cout << ">>> ΑΠΟΤΥΧΙΑ: Ακόμα και χωρίς περιορισμούς το σημείο είναι μάλλον εκτός εμβέλειας του βραχίονα ή χτυπάει στο πάτωμα." << std::endl;
        }
        
        // Καθαρίζουμε τον στόχο για να είναι έτοιμος για τις επόμενες συντεταγμένες
        arm_interface.clearPoseTargets();
    }
  }

  rclcpp::shutdown();
  return 0;
}
